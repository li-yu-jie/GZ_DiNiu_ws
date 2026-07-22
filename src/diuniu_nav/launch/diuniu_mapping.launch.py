# =============================================================================
# diuniu_mapping.launch.py — 地牛叉车建图模式一键启动文件（Web 端建图使用）
#
# 启动组件：
#   1. livox_ros_driver2 — Mid360 雷达驱动
#   2. fast_lio — FAST-LIO SLAM（退出时自动落盘 PCD：src/FAST_LIO/PCD/scans.pcd）
#   3. diuniu_base — 底盘驱动（关闭自身 odom TF/topic，避免与 FAST-LIO 冲突）
#   4. robot_state_publisher — URDF 静态 TF
#   5. pointcloud_to_laserscan + laserscan_filter — 供 Web 端实时显示干净激光
#
# 与 diuniu_nav_all.launch.py 的区别：不启动 Nav2 / AMCL / map_server，
# 建图期间网页用虚拟摇杆 (/cmd_vel_joy) 遥控，完成后由 Web 后端执行保存流水线。
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

    # 5. 3D 点云 → 2D 激光切片（参数与 diuniu_nav.launch.py 保持一致）
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': 0.10,
            'max_height': 1.2,
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

    # 6. 雷达自遮挡过滤器：输出干净的 /scan_filtered 供 Web 端显示
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
        livox_launch,
        fast_lio_launch,
        base_launch,
        robot_state_publisher_node,
        pointcloud_to_laserscan,
        laserscan_filter
    ])
