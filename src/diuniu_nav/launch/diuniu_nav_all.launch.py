# =============================================================================
# diuniu_nav_all.launch.py — 地牛叉车一键启动全部导航相关节点
#
# 本文件按顺序启动实车自主导航所需的四个核心组件：
#   1. livox_ros_driver2 — Mid360 雷达驱动
#   2. fast_lio — FAST-LIO SLAM 里程计
#   3. diuniu_base — 底盘驱动（关闭自身 odom TF/topic，避免与 FAST-LIO 冲突）
#   4. diuniu_nav — Nav2 导航（模式一 B：FAST-LIO + AMCL 地图匹配重定位）
#
# 启动前请确认：
#   - 已切换到 ~/GZ_DiNiu_ws 并 source install/setup.bash
#   - 雷达、底盘串口已正确连接
#   - 车辆周围无障碍物，处于安全状态
#
# 用法：
#   ros2 launch diuniu_nav diuniu_nav_all.launch.py
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_diuniu_nav = get_package_share_directory('diuniu_nav')
    pkg_livox = get_package_share_directory('livox_ros_driver2')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    pkg_diuniu_base = get_package_share_directory('diuniu_base')

    # 模式切换参数：默认使用 FAST-LIO + AMCL 地图匹配重定位
    use_relocalization = LaunchConfiguration('use_relocalization')
    declare_use_relocalization = DeclareLaunchArgument(
        'use_relocalization',
        default_value='true',
        description='true=模式一 B（FAST-LIO + AMCL 地图匹配）；false=模式一 A（静态原点）'
    )

    # 1. 启动 Mid360 雷达驱动
    # livox_ros_driver2 的 ROS2 launch 文件安装在 share/<pkg>/launch_ROS2/ 目录下
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_livox, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # 2. 启动 FAST-LIO SLAM（不自动打开 RViz）
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 3. 启动底盘驱动（关闭底盘自身 odom TF 和 odom topic，由 FAST-LIO 提供）
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diuniu_base, 'launch', 'diuniu_base.launch.py')
        ),
        launch_arguments={
            'pub_odom_tf': 'false',
            'pub_odom_topic': 'false'
        }.items()
    )

    # 4. 启动导航（模式一，默认带地图匹配重定位）
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_diuniu_nav, 'launch', 'diuniu_nav.launch.py')
        ),
        launch_arguments={
            'use_amcl': 'false',
            'use_relocalization': use_relocalization
        }.items()
    )

    return LaunchDescription([
        declare_use_relocalization,
        livox_launch,
        fast_lio_launch,
        base_launch,
        nav_launch
    ])
