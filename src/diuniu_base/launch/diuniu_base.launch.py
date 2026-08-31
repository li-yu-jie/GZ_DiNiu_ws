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
            default_value='false',
            description='Whether to publish odom TF（默认 false：实车由 FAST-LIO/EKF 发布，'
                        '默认 true 单独启动时会与 FAST-LIO 双重发布冲突；仅无 SLAM 的纯底盘调试时设 true）'
        ),
        DeclareLaunchArgument(
            'pub_odom_topic',
            default_value='false',
            description='Whether to publish odom topic（默认 false：避免与 FAST-LIO /odom 冲突；'
                        '仅无 SLAM 的纯底盘调试时设 true，/wheel_odom 始终发布不受影响）'
        ),

        DeclareLaunchArgument(
            'steer_rate_limit_dps',
            default_value='240.0',
            description='转向角速率限制 (度/秒)，防止单周期满舵跳变'
        ),

        DeclareLaunchArgument(
            'odom_vx_scale',
            default_value='0.936',
            description='轮式里程计线速度比例系数（2026-08-29 FAST-LIO 雷达真值标定：'
                        '0.2/0.4/0.6m/s × 1m/2m 六组 k≈0.936 与速度无关，纯比例误差）'
        ),
        DeclareLaunchArgument(
            'odom_vx_slip',
            default_value='0.0',
            description='轮式里程计速度滑移系数（2026-08-29 雷达标定证实与速度无关，置 0）'
        ),

        Node(
            package='diuniu_base',
            executable='diuniu_base',
            name='diuniu_base',
            parameters=[{
                'serial_port': LaunchConfiguration('port'),
                'baud_rate': LaunchConfiguration('baud'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'steer_rate_limit_dps': LaunchConfiguration('steer_rate_limit_dps'),
                'odom_vx_scale': LaunchConfiguration('odom_vx_scale'),
                'odom_vx_slip': LaunchConfiguration('odom_vx_slip'),
                'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
                'pub_odom_topic': LaunchConfiguration('pub_odom_topic'),
            }],
            output='screen',
        ),
    ])
