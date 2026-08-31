# =============================================================================
# diuniu_nav.launch.py — 地牛叉车 Nav2 自主导航一键启动文件
#
# 支持三种定位模式（通过 use_amcl / use_relocalization 参数切换）：
#   模式 A（use_amcl:=false）：FAST-LIO 直连 SLAM 高精定位导航
#       - 前提：需先启动 livox_ros_driver2 + fast_lio（提供 odom→base_link TF 和 /odom）
#       - ⚠️ FAST-LIO 发布的 odom→base_link 实为雷达(IMU)位姿，与诚实 base_link 相差
#         常值 (1.215, 0, 0.66)；2026-08-28 坐标系诚实化后模式 A 与 /scan 存在该偏差，
#         仅保留用于调试，实车导航请用模式 B（AMCL + EKF）
#       - use_relocalization:=false（默认）：发布静态 TF map→odom（单位变换，要求建图原点即导航起点）
#       - use_relocalization:=true：启动 AMCL，用 /scan_filtered 与 2D 栅格地图匹配，动态发布 map→odom
#   模式 B（use_amcl:=true）：AMCL 纯定位导航【当前默认，见 declare_use_amcl 默认值】
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
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    use_ekf = LaunchConfiguration('use_ekf')
    ekf_config_file = os.path.join(pkg_nav, 'config', 'ekf.yaml')

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

    # ★ bt_navigator 的 odom_topic 按定位模式注入，保证与 TF 来源一致：
    #    use_ekf:=false → /odom（FAST-LIO 原始输出）；use_ekf:=true → /odometry/filtered（EKF 融合输出）
    #    ⚠️ 必须用 PythonExpression 运行时求值，不能用 Python == 比较 LaunchConfiguration 对象
    bt_odom_topic = PythonExpression(
        ["'/odometry/filtered' if '", LaunchConfiguration('use_ekf'), "' == 'true' else '/odom'"])
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': default_bt_xml,
            'default_bt_xml_filename': default_bt_xml,
            'default_nav_through_poses_bt_xml': default_bt_xml_through_poses,
            'odom_topic': bt_odom_topic,
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
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='true=启动 robot_localization EKF（轮式里程计 vx/wz + BNO085 绝对航向，'
                    '发布 odom→base_link）；false=用 FAST-LIO 原始 /odom（航向为雷达上电朝向，仅调试）')

    # EKF 传感器融合节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_ekf)
    )

    # 机器人状态发布节点：根据 URDF 发布 base_link 到各关节/传感器的静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # ======================== 模式 B：AMCL 定位（use_amcl:=true） ========================
    # 等效于 nav2_bringup bringup_launch.py（= localization + navigation），
    # 但 navigation 部分改用 diuniu_navigation_launch.py（含 collision_monitor 急停链）
    bringup_with_amcl = GroupAction(
        condition=IfCondition(use_amcl),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav_bringup, 'launch', 'localization_launch.py')),
                launch_arguments={
                    'map': map_yaml,
                    'params_file': configured_params,
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'diuniu_navigation_launch.py')),
                launch_arguments={
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
            # ★ 使用 diuniu_navigation_launch.py（含 collision_monitor 急停链），
            #    勿改回 nav2_bringup 原版 navigation_launch.py（无急停层）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'diuniu_navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'params_file': configured_params
                }.items()
            )
        ]
    )

    # ============ 禁区层已拆除（2026-08-27） ============
    # 背景：旧 keepout_mask 对齐的是已作废的旧地图（origin 差十几米），新地图重录后
    # mask 未重画，launch 若继续加载旧 mask 会把禁区贴到错误位置（比没有更危险）。
    # 已从本文件移除 filter_mask_server / costmap_filter_info_server /
    # lifecycle_manager_keepout 三个节点，nav2_params.yaml 同步摘除 keepout_filter。
    # 如需恢复禁区：按新地图重画 keepout_mask.pgm/yaml 放回 maps/，再恢复上述节点
    # （git 历史可查到原始配置）。

    # ============ 点云水平补偿节点（两种模式都启动） ============
    # 用 FAST-LIO /odom 的重力对齐姿态实时消除桅杆摆动的 roll/pitch，再交给切片。
    # 不补偿时支架晃动会把头顶横梁/门架压进切片 → 前方"幽灵障碍"（2026-08-24 实车事故根因）
    cloud_level = Node(
        package='diuniu_base',
        executable='cloud_level',
        name='cloud_level_node',
        parameters=[{
            'cloud_in': '/cloud_registered_body',
            'cloud_out': '/cloud_leveled',
            'odom_topic': '/odom',
        }],
        output='screen'
    )

    # ============ 3D 点云 → 2D 激光切片节点（两种模式都启动） ============
    # 将补偿后的水平点云压扁成 2D LaserScan，供代价地图避障与 AMCL 使用
    # ★ 2026-08-28 起 /cloud_leveled 是【诚实的 base_link 系】（cloud_level_node 已完成
    #    雷达系→base_link 平移，z=0 在地面），高度区间直接按地面系书写：
    #      z∈[0.20, 1.20]m —— 与地图建图切片层严格一致，可看到托盘/纸箱/脚踝。
    #    ⚠️ 雷达若再移位须同步改 cloud_level 的 base_offset_* 与 URDF，否则全链错位。
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_leveled'),  # 水平补偿+坐标系诚实化后的点云（frame: base_link）
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.5,   # ★ 放宽至 0.5s：与 costmap 的 transform_tolerance 统一，彻底解决 TF 抖动/延迟导致的丢帧警告
            'min_height': 0.20,       # ★ 地面系 z（z=0 在地面）：切除地面 20cm 以下光斑杂波
            'max_height': 1.20,       # ★ 地面系 z：切除 1.2m 以上空中障碍物与顶棚
            'angle_min': -3.1415926,  # 全周 360° 扫描
            'angle_max': 3.1415926,
            'angle_increment': 0.0087,  # 角分辨率约 0.5°
            'scan_time': 0.1,
            'range_min': 0.15,        # ★ 0.15m 保留近场侧向感知（collision_monitor 用）；
                                      #    坐标系诚实化后雷达罩/桅杆近场自反射点落在车身包络内，
                                      #    由 laserscan_filter 屏蔽盒统一滤除，不再依赖 range_min 0.50
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
            'concurrency_level': 0,
            'queue_size': 2           # ★ 安全链取最新帧：队列>2 会在 CPU 争抢时积压旧点云，
                                      #    障碍进入 /scan 的延迟最坏 = 队列×100ms（10Hz）
        }],
        output='screen'
    )

    # ============ 雷达自遮挡过滤器（两种模式都启动） ============
    # 屏蔽盒 2026-08-28 二次收紧（用户实车确认几何：货叉从车尾向前 1.20m 伸在车身
    #   范围内、车尾无伸出，载货按车身长宽计）：
    #   x_min=-0.35：车身后界 -0.30m+5cm 余量。旧值 -1.65 基于"叉尖伸出车尾"的
    #     错误假设，会把车尾 1.3m 内的真实行人/障碍一并删掉（倒车时致命），已修正。
    #   x_max=1.65 / y=±0.36：车头前边界 1.60m+5cm / 半宽 0.35m+1cm。
    #   历史：x_max=2.60/y=±1.60 是【1.6m 倒装桅杆雷达】斜打前罩/门架油缸(1.87~2.58m)与
    #   斜前 48° 地面的鬼影环经验裁切值（git 2eed3e6/c8a0a5f/5da4e7c）——代价地图订的是
    #   /scan_filtered，宽盒会把车身两侧 1.25m 内、车头 1.65~2.6m 的行人全部滤掉
    #   （"不避障行人"根因，2026-08-28 实车确认后收紧）。
    # ⚠️ 若行驶中车前/斜前再发鬼影环，先查 cloud_level 链与雷达时钟失锁，勿直接放宽盒子。
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.35,           # ★ 车身后界 -0.30m+5cm（车尾无伸出，勿再退回 -1.65 幻影尾巴）
            'x_max': 1.65,            # ★ 车头物理前边界 1.60m + 5cm 余量 = 1.65
            'y_min': -0.36,           # ★ 2026-08-28 从 -0.40 收紧至 -0.36（车身半宽 0.35m+1cm 余量）
                                      #    原 ±0.40 过滤盒比车体宽 5cm，紧贴车侧的人会被误删 → 不避障
                                      #    ±0.36 仅覆盖车体物理轮廓，不会误删侧向行人
            'y_max': 0.36,
            'laser_x_offset': 0.0,    # ★ 0.0m：pointcloud_to_laserscan target_frame 已是 base_link，无须二次叠加偏移！
            'laser_y_offset': 0.0,
            'arc_filter_enabled': False  # ★ 2026-08-28 关闭：圆弧过滤是 1.6m 倒装桅杆期产物，
                                         #    会把斜前 ±38°~62°、2.2m 内的真实行人一并删掉
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_use_amcl,
        declare_use_relocalization,
        declare_use_ekf,
        robot_state_publisher_node,
        ekf_node,
        bringup_with_amcl,
        bringup_without_amcl,
        cloud_level,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
