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
from launch.actions import IncludeLaunchDescription
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
    #    ⚠️ 高度区间是【雷达系 z】（点云原点在 1.6m 高的雷达处）：
    #       z∈[-1.40, 0.0] = 地面以上 [0.20, 1.60]m，与 2D 地图建图切片层一致
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
            'transform_tolerance': 0.2,
            'min_height': -1.30,      # 雷达系 z（原点在 1.6m）：地面 +0.30m
            'max_height': -0.60,      # 雷达系 z（原点在 1.6m）：地面 +1.00m
            'angle_min': -3.1415926,
            'angle_max': 3.1415926,
            'angle_increment': 0.0087,
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

    # 7. 雷达自遮挡过滤器：输出干净的 /scan_filtered 供 Web 端显示
    #    参数与 diuniu_nav.launch.py 保持一致（x∈[-1.65, 1.60]，覆盖 footprint + 货叉载货区）
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -1.65,
            'x_max': 1.60,
            'y_min': -0.45,
            'y_max': 0.45,
            'laser_x_offset': 0.0,
            'laser_y_offset': 0.0
        }],
        output='screen'
    )

    return LaunchDescription([
        livox_launch,
        fast_lio_launch,
        base_launch,
        robot_state_publisher_node,
        cloud_level,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
