# =============================================================================
# n10_nav_all.launch.py — N10 雷达 Nav2 自主导航一键启动（AMCL 定位）
#
# 启动组件：
#   1. lslidar_driver — N10 驱动 → /scan (n10_laser_link)
#   2. robot_state_publisher — URDF 静态 TF
#   3. diuniu_base — 底盘（/wheel_odom + /imu/data 常发；odom TF/topic 关闭）
#   4. EKF — 轮程 vx/wz + BNO085 航向 → odom→base_link TF（N10 栈唯一来源，必选）
#   5. laserscan_filter — /scan → /scan_filtered（供 AMCL / costmap）
#   6. nav2 localization_launch.py — map_server + AMCL（map→odom TF）
#   7. diuniu_nav/diuniu_navigation_launch.py — 导航核心 + collision_monitor 急停链
#      （cmd_vel_nav → collision_monitor → cmd_vel_cm → velocity_smoother → cmd_vel）
#
# 与 diuniu_nav 栈的关系：完全独立，不动 diuniu_nav 任何文件；
# Mid360 + FAST-LIO 栈（diuniu_nav_all.launch.py）保留可回退。
#
# ⚠️ 预飞纪律（与 Mid360 栈相同）：
#   RViz 里确认激光贴墙 + AMCL 粒子收敛后再发导航目标；
#   定量检查可跑 tools/scan_map_fit.py。初始位姿点错会撞墙（2026-08-28 事故）。
#
# 用法：
#   ros2 launch diuniu_n10_nav n10_nav_all.launch.py
#   ros2 launch diuniu_n10_nav n10_nav_all.launch.py map:=/path/to/other_map.yaml
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_n10 = get_package_share_directory('diuniu_n10_nav')
    pkg_nav = get_package_share_directory('diuniu_nav')
    pkg_nav_bringup = get_package_share_directory('nav2_bringup')
    pkg_diuniu_base = get_package_share_directory('diuniu_base')
    pkg_description = get_package_share_directory('diuniu_description')

    urdf_file = os.path.join(pkg_description, 'urdf', 'diuniu.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    default_map = os.path.join(pkg_n10, 'maps', 'map.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map, description='2D 栅格地图 yaml 完整路径')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='是否使用仿真时间（实车必须为 false）')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/n10_lidar', description='N10 串口设备')

    # ★ 自定义行为树（无 Spin 版）复用 diuniu_nav 的 xml；
    #   bt_navigator 只解析 nav2_bt_navigator 自家目录的相对文件名，必须注入绝对路径。
    #   odom_topic 固定 /odometry/filtered（EKF 融合输出，N10 栈 EKF 恒开）。
    default_bt_xml = os.path.join(
        pkg_nav, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery_no_spin.xml')
    default_bt_xml_through_poses = os.path.join(
        pkg_nav, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery_no_spin.xml')
    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_n10, 'config', 'nav2_params.yaml'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': default_bt_xml,
            'default_bt_xml_filename': default_bt_xml,
            'default_nav_through_poses_bt_xml': default_bt_xml_through_poses,
            'odom_topic': '/odometry/filtered',
        },
        convert_types=True
    )

    # 1. N10 驱动
    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg_n10, 'config', 'lsx10.yaml'),
                    {'serial_port_': serial_port}],
    )

    # 2. URDF 静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # 3. 底盘驱动（关闭自带 odom TF/topic，由 EKF 统一发布；/wheel_odom 常发）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diuniu_base, 'launch', 'diuniu_base.launch.py')
        ),
        launch_arguments={
            'pub_odom_tf': 'false',
            'pub_odom_topic': 'false'
        }.items()
    )

    # 4. EKF：轮程 vx/wz + BNO085 绝对航向 → odom→base_link TF（N10 栈必选）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_n10, 'config', 'ekf.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # 5. 雷达自遮挡过滤器（参数与 diuniu_nav 保持一致，改动请两边同步）
    #    ★ laser_x_offset=1.295：过滤器不走 TF，直接在扫描坐标系算 x_base=x_laser+offset
    #      （Mid360 链 /scan 已是 base_link 系故 offset=0；N10 的 /scan 是 n10_laser_link
    #       系，必须给雷达在 base_link 下的 x 偏移(原雷达位前方 8cm)，否则屏蔽盒错位滤不到车身）
    #    ⚠️ 过滤器不支持旋转补偿：N10 安装 yaw 必须物理对准车头（URDF n10_laser_joint yaw=0）
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.35,
            'x_max': 1.95,
            'y_min': -0.36,
            'y_max': 0.36,
            'laser_x_offset': 1.295,   # ★ N10 在 base_link 前方 1.295m（原雷达位前 8cm，与 URDF 同步）
            'laser_y_offset': 0.0,
            'arc_filter_enabled': False
        }],
        output='screen'
    )

    # 6. 定位：map_server + AMCL（scan_topic: scan_filtered，见 nav2_params.yaml）
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    # 7. 导航核心：复用 diuniu_navigation_launch.py（含 collision_monitor 急停链），
    #    勿改回 nav2_bringup 原版 navigation_launch.py（无急停层）
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'diuniu_navigation_launch.py')),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_serial_port,
        driver_node,
        robot_state_publisher_node,
        base_launch,
        ekf_node,
        laserscan_filter,
        localization_launch,
        navigation_launch,
    ])
