# =============================================================================
# diuniu_mapping.launch.py — 地牛叉车建图模式一键启动文件
#
# 启动组件：
#   1. livox_ros_driver2 — Mid360 雷达驱动
#   2. fast_lio — FAST-LIO SLAM（退出时自动落盘 PCD：src/FAST_LIO/PCD/scans.pcd）
#   3. diuniu_base — 底盘驱动（关闭自身 odom TF/topic，避免与 FAST-LIO 冲突）
#   4. robot_state_publisher — URDF 静态 TF
#   5. pointcloud_to_laserscan + laserscan_filter — 输出干净的 /scan_filtered
#
# 与 diuniu_nav_all.launch.py 的区别：不启动 Nav2 / AMCL / map_server。
# 建图期间用手柄遥控（ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py），
# 完成后 Ctrl+C 退出，FAST-LIO 自动将点云落盘为 PCD，再用 pcd2pgm 包转 2D 地图。
#
# 用法：
#   ros2 launch diuniu_nav diuniu_mapping.launch.py
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_livox = get_package_share_directory('livox_ros_driver2')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_diuniu_base = get_package_share_directory('diuniu_base')
    pkg_description = get_package_share_directory('diuniu_description')

    # 整车 URDF：发布 base_link→各传感器静态 TF（点云切片与 Web 显示需要）
    urdf_file = os.path.join(pkg_description, 'urdf', 'diuniu.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # LaunchConfiguration 使用前必须声明，否则直接运行本 launch 报"参数未声明"
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Mid360 雷达驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_livox, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. FAST-LIO SLAM（不打开 RViz；pcd_save_en=true，收到 SIGINT 退出时自动存 PCD）
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 3. 底盘驱动（关闭自带里程计，由 FAST-LIO 提供 /odom 与 TF）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diuniu_base, 'launch', 'diuniu_base.launch.py')
        ),
        launch_arguments={
            'pub_odom_tf': 'false',
            'pub_odom_topic': 'false'
        }.items()
    )

    # 4. URDF 静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]
    )

    # 5. 点云水平补偿（参数与 diuniu_nav.launch.py 保持一致，改动请两边同步）
    #    消除桅杆摆动 roll/pitch，防止建图时切片层被姿态抖动污染
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

    # 6. 3D 点云 → 2D 激光切片（参数与 diuniu_nav.launch.py 保持一致，改动请两边同步）
    #    ★ 2026-08-28 起 /cloud_leveled 是【诚实的 base_link 系】（z=0 在地面）：
    #       z∈[0.20, 1.20]m = 地面以上切片带，与 2D 地图建图切片层一致
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_leveled'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.5,   # ★ 放宽至 0.5s：与 costmap 的 transform_tolerance 统一，彻底解决 TF 抖动/延迟导致的丢帧警告
            'min_height': 0.20,       # ★ 地面系 z（z=0 在地面）：切除地面 20cm 以下光斑杂波
            'max_height': 1.20,       # ★ 地面系 z：切除 1.2m 以上空中障碍物与顶棚
            'angle_min': -3.1415926,
            'angle_max': 3.1415926,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.15,        # ★ 与 nav launch 同步：0.15m 近场自反射由屏蔽盒统一滤除
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
            'concurrency_level': 0,
            'queue_size': 2           # ★ 与 nav launch 同步：取最新帧，防积压旧点云
        }],
        output='screen'
    )

    # 7. 雷达自遮挡过滤器：输出干净的 /scan_filtered 供 Web 端显示
    #    参数与 diuniu_nav.launch.py 保持一致（改动请两边同步）：
    #    x_min=-0.35 车身后界+5cm（货叉在车身范围内、车尾无伸出，8-28 用户实车确认；
    #    旧值 -1.65 系"叉尖伸出车尾"错误假设，会删掉车尾真实障碍）；x_max=1.65/y=±0.36 车身轮廓+1cm
    #    （2026-08-28 收紧：0.66m 正装后实测无旧倒装雷达的鬼影环；
    #      ±0.40 会把紧贴车侧的行人误删 → 不避障，故统一收紧到 ±0.36）
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.35,
            'x_max': 1.65,
            'y_min': -0.36,
            'y_max': 0.36,
            'laser_x_offset': 0.0,
            'laser_y_offset': 0.0,
            'arc_filter_enabled': False  # ★ 2026-08-28 关闭（桅杆期产物，会删斜前 2.2m 内真实行人）
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        livox_launch,
        fast_lio_launch,
        base_launch,
        robot_state_publisher_node,
        cloud_level,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
