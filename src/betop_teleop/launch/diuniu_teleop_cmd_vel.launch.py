#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘话题中转手柄节点启动文件
===================================
使用方法：
  ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ──────────────────────────────────────────────────────
        # 节点 1：diuniu_joy_publisher (自定义手柄驱动)
        # 直接读取 /dev/input/js0，发布 sensor_msgs/Joy 到 /joy 话题
        # 用于绕过 ros-humble-joy 的 joy_node 对北通 BTP-KP20D
        # 报 "sequence size exceeds remaining buffer" 的解析问题。
        # ──────────────────────────────────────────────────────
        Node(
            package='betop_teleop',
            executable='diuniu_joy_publisher',
            name='diuniu_joy_publisher',
            parameters=[{
                'device': '/dev/input/js0',
                'publish_rate': 50.0,
                'num_axes': 8,
                'num_buttons': 16,
            }],
            output='screen',
        ),

        # ──────────────────────────────────────────────────────
        # 节点 2：diuniu_teleop_cmd_vel (话题发布)
        # ──────────────────────────────────────────────────────
        Node(
            package='betop_teleop',
            executable='diuniu_teleop_cmd_vel',
            name='diuniu_teleop_cmd_vel',
            parameters=[{
                'max_linear_speed': 1.2,
                'max_angular_speed': 2.5,
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
