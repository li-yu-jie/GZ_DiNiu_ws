# =============================================================================
# diuniu_nav.launch.py — 地牛叉车 Nav2 自主导航一键启动文件
#
# 支持三种定位模式（通过 use_amcl / use_relocalization 参数切换）：
#   模式 A（use_amcl:=false）：FAST-LIO 直连 SLAM 高精定位导航【默认实车模式】
#       - 前提：需先启动 livox_ros_driver2 + fast_lio（提供 odom→base_link TF 和 /odom）
#       - use_relocalization:=false（默认）：发布静态 TF map→odom（单位变换，要求建图原点即导航起点）
#       - use_relocalization:=true：启动 AMCL，用 /scan_filtered 与 2D 栅格地图匹配，动态发布 map→odom
#   模式 B（use_amcl:=true，默认）：AMCL 纯定位导航
#       - 使用 nav2_bringup 标准 bringup（map_server + AMCL + navigation）
#
# ⚠️ 重要前提（两种模式都需要）：
#   1. 底盘节点 diuniu_base_node 必须单独启动（本文件不含底盘驱动）
#   2. 模式 A 下启动底盘时必须关闭其自带里程计发布，避免与 FAST-LIO 双重发布冲突：
#        ros2 launch diuniu_base diuniu_base.launch.py pub_odom_tf:=false pub_odom_topic:=false
#   3. 不要重复启动同一个 launch，同名节点冲突会导致 TF / costmap 异常
#
# 常用启动命令：
#   ros2 launch diuniu_nav diuniu_nav.launch.py                                         # 模式 B（AMCL）
#   ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false                         # 模式 A-1（FAST-LIO 静态原点）
#   ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false use_relocalization:=true # 模式 A-2（FAST-LIO + AMCL 重定位）
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_nav = get_package_share_directory('diuniu_nav')
    pkg_nav_bringup = get_package_share_directory('nav2_bringup')
    pkg_description = get_package_share_directory('diuniu_description')

    # 加载整车 URDF 模型，供 robot_state_publisher 发布 base_link→各传感器/轮子的静态 TF
    urdf_file = os.path.join(pkg_description, 'urdf', 'diuniu.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 默认文件路径：2D 栅格地图 + Nav2 参数文件
    default_map = os.path.join(pkg_nav, 'maps', 'map.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Launch 配置项（运行时可通过 xxx:=yyy 覆盖）
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')
    use_relocalization = LaunchConfiguration('use_relocalization')

    # ★ 自定义行为树（已移除 Spin 90° 恢复动作，防 Tricycle 伪原地自转扫墙/撞墙转圈）
    # bt_navigator 只会在 nav2_bt_navigator 自家目录解析相对文件名，
    # 因此这里用 RewrittenYaml 在启动时把绝对路径注入参数文件（生成临时 yaml）
    # ⚠️ to_pose 与 through_poses 两棵树都必须换成无 Spin 版：
    #    behavior_server 已禁用 spin 插件，默认 through_poses 树里的 <Spin>
    #    会让 bt_navigator 激活失败（"spin action server not available"），
    #    导致整车导航目标被全部拒绝
    default_bt_xml = os.path.join(
        pkg_nav, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery_no_spin.xml')
    default_bt_xml_through_poses = os.path.join(
        pkg_nav, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery_no_spin.xml')
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': default_bt_xml,
            'default_bt_xml_filename': default_bt_xml,
            'default_nav_through_poses_bt_xml': default_bt_xml_through_poses,
        },
        convert_types=True
    )

    # 声明启动参数
    declare_map = DeclareLaunchArgument('map', default_value=default_map, description='2D 栅格地图 yaml 完整路径')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params, description='nav2 参数文件完整路径')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间（实车必须为 false）')
    declare_use_amcl = DeclareLaunchArgument('use_amcl', default_value='true', description='true=模式B(AMCL定位)；false=模式A(FAST-LIO直连定位)')
    declare_use_relocalization = DeclareLaunchArgument(
        'use_relocalization',
        default_value='false',
        description='true=在模式A中启动AMCL进行2D地图匹配重定位；false=模式A使用静态map→odom（需从建图原点启动）')

    # 机器人状态发布节点：根据 URDF 发布 base_link 到各关节/传感器的静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # ======================== 模式 B：AMCL 定位（use_amcl:=true） ========================
    # 直接使用 nav2_bringup 的标准 bringup：map_server + AMCL + 全套导航节点
    bringup_with_amcl = GroupAction(
        condition=IfCondition(use_amcl),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_yaml,
                    'params_file': configured_params,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # ================== 模式 A：FAST-LIO 直连定位（use_amcl:=false） ==================
    # 定位由外部 FAST-LIO 提供（odom→base_link TF + /odom 话题）
    # 根据 use_relocalization 决定是静态 map→odom 还是 AMCL 地图匹配动态 map→odom
    bringup_without_amcl = GroupAction(
        condition=UnlessCondition(use_amcl),
        actions=[
            # ---------- 模式 A-2：AMCL 地图匹配重定位 ----------
            # 用 /scan_filtered 与 2D 栅格地图匹配，动态发布 map→odom
            # 适合开机位置不在建图原点的场景
            GroupAction(
                condition=IfCondition(use_relocalization),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'localization_launch.py')),
                        launch_arguments={
                            'map': map_yaml,
                            'params_file': configured_params,
                            'use_sim_time': use_sim_time,
                            'autostart': 'true'
                        }.items()
                    )
                ]
            ),
            # ---------- 模式 A-1：纯 FAST-LIO 静态原点 ----------
            # 发布静态 map→odom（单位变换），要求开机位置/航向与建图原点一致
            GroupAction(
                condition=UnlessCondition(use_relocalization),
                actions=[
                    # 1. 地图服务器：加载并发布 2D 栅格地图
                    Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        output='screen',
                        parameters=[configured_params, {'yaml_filename': map_yaml}]
                    ),
                    # 2. 生命周期管理器：自动激活 map_server
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_localization',
                        output='screen',
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': True},
                                    {'node_names': ['map_server']}]
                    ),
                    # 3. 静态 TF map→odom（单位变换）：认为建图原点就是地图原点
                    #    ⚠️ 若建图起点与地图原点不一致，需修改此处平移/旋转参数
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_tf_map_to_odom',
                        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
                    )
                ]
            ),
            # ---------- 导航核心：planner / controller / behavior / bt_navigator ----------
            # 两种子模式共用
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'params_file': configured_params
                }.items()
            )
        ]
    )

    # ============ 3D 点云 → 2D 激光切片节点（两种模式都启动） ============
    # 将 FAST-LIO 输出的机体坐标系点云压扁成 2D LaserScan，供代价地图避障与 AMCL 使用
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),  # FAST-LIO 机体系点云（frame: base_link）
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.10,       # ★ 设为 10cm！全面捕捉前方人体脚踝、鞋子与低矮障碍物，绝不漏扫撞人
            'max_height': 1.2,        # 切片上限 1.2m，覆盖常见货架/人腿高度
            'angle_min': -3.1415926,  # 全周 360° 扫描
            'angle_max': 3.1415926,
            'angle_increment': 0.0087,  # 角分辨率约 0.5°
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
            'concurrency_level': 0,
            'queue_size': 2
        }],
        output='screen'
    )

    # ============ 雷达自遮挡过滤器（两种模式都启动） ============
    # 剔除 base_link 系下车体自身与前货叉的反射点，输出干净的 /scan_filtered
    # 过滤区域：x∈[-0.25, 1.60]m（覆盖车尾到货叉中部），y∈[-0.35, 0.35]m（车宽 0.7m）
    # ⚠️ x_max 必须与 nav2_params.yaml 中 footprint 前边界保持一致，否则会留下避障盲区
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.25,
            'x_max': 1.30,
            'y_min': -0.35,
            'y_max': 0.35,
            'laser_x_offset': 0.0,
            'laser_y_offset': 0.0
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_use_amcl,
        declare_use_relocalization,
        robot_state_publisher_node,
        bringup_with_amcl,
        bringup_without_amcl,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
