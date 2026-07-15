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
