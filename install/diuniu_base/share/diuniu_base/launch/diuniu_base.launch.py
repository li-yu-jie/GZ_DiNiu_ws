#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘驱动节点启动文件
============================
使用方法：
  ros2 launch diuniu_base diuniu_base.launch.py
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
            description='底盘串口设备路径'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='460800',
            description='串口通信波特率'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='1.30',
            description='机器人物理轴距 (m)'
        ),

        DeclareLaunchArgument(
            'pub_odom_tf',
            default_value='true',
            description='Whether to publish odom TF'
        ),
        DeclareLaunchArgument(
            'pub_odom_topic',
            default_value='true',
            description='Whether to publish odom topic'
        ),

        Node(
            package='diuniu_base',
            executable='diuniu_base',
            name='diuniu_base',
            parameters=[{
                'serial_port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
                'pub_odom_topic': LaunchConfiguration('pub_odom_topic'),
            }],
            output='screen',
        ),
    ])
