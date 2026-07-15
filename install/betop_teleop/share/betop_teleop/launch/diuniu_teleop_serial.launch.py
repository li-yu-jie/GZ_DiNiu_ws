#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘串口直控手柄节点启动文件
===================================
使用方法：
  ros2 launch betop_teleop diuniu_teleop_serial.launch.py port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='底盘串口物理设备路径'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='460800',
            description='串口通信波特率'
        ),

        # ──────────────────────────────────────────────────────
        # 节点 1：joy_node (手柄驱动)
        # ──────────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 10.0,
            }],
            output='screen',
        ),

        # ──────────────────────────────────────────────────────
        # 节点 2：diuniu_teleop_serial (串口直控)
        # ──────────────────────────────────────────────────────
        Node(
            package='betop_teleop',
            executable='diuniu_teleop_serial',
            name='diuniu_teleop_serial',
            parameters=[{
                'serial_port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud'),
                'max_linear_speed': 1.2,
                'max_steer_angle': 95.0,  # 打角极限度数
                'axis_linear': 1,
                'axis_steer': 2,
                'enable_button': 7,
                'stop_button': 1,
                'button_lift_up': 4,
                'button_lift_down': 0,
                'steer_invert': False,
                'publish_rate': 20.0,
            }],
            output='screen',
        ),
    ])
