# =============================================================================
# n10_driver.launch.py — 镭神 LSLiDAR N10 驱动单独启动（调试用）
#
# 发布 /scan (sensor_msgs/LaserScan, frame_id: n10_laser_link)。
# 串口默认 /dev/n10_lidar（udev 符号链接，规则见 config/99-diuniu.rules）；
# 规则未安装前可用 serial_port:=/dev/ttyUSB1 临时指定。
#
# 用法：
#   ros2 launch diuniu_n10_nav n10_driver.launch.py
#   ros2 launch diuniu_n10_nav n10_driver.launch.py serial_port:=/dev/ttyUSB1
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_n10 = get_package_share_directory('diuniu_n10_nav')
    params_file = os.path.join(pkg_n10, 'config', 'lsx10.yaml')

    serial_port = LaunchConfiguration('serial_port')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/n10_lidar',
        description='N10 串口设备（建议用 udev 符号链接 /dev/n10_lidar，勿用会漂移的 ttyUSBx 编号）')

    # 驱动是普通 rclcpp::Node（非生命周期节点），用 Node 启动即可
    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {'serial_port_': serial_port}],
    )

    return LaunchDescription([
        declare_serial_port,
        driver_node,
    ])
