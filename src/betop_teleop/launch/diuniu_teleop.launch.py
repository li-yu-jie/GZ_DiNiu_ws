#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘遥控系统启动文件
============================
同时启动以下两个节点：
  1. joy_node       —— 读取手柄（/dev/input/js0），发布 /joy 话题
  2. diuniu_teleop  —— 订阅 /joy，将摇杆/按键映射为串口指令控制底盘

使用方法：
  ros2 launch betop_teleop diuniu_teleop.launch.py

若手柄设备号不是 js0，可在本文件中修改 joy_node 的 device_id 参数。
若要修改按键映射或速度上限，直接修改下方 diuniu_teleop 的参数即可。
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
        # 若要修改手柄设备路径，可在下方 'device' 参数中指定。
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
        # 节点 2：diuniu_teleop
        # 功能：订阅 /joy 话题，将摇杆/按键数据映射为串口 ASCII 指令，
        #        通过 /dev/ttyUSB0 发送给 STM32 底盘控制器
        # ──────────────────────────────────────────────────────
        Node(
            package='betop_teleop',
            executable='diuniu_teleop',
            name='diuniu_teleop',
            parameters=[{
                # ── 串口配置 ──────────────────────────────────
                # STM32 连接的串口设备路径
                'serial_port': '/dev/ttyUSB0',
                # 串口波特率，需与 STM32 固件中 UART 初始化一致
                'baud_rate': 460800,

                # ── 速度 / 转向映射 ───────────────────────────
                # 最大线速度（m/s）：左摇杆满偏时对应的目标速度
                'max_linear_speed': 1.2,
                # 最大转向脉冲数：右摇杆满偏时对应的转向步进脉冲
                'max_steer_pulse': 310000,

                # ── 轴编号（北通 BFM 模式实测）────────────────
                # 线速度控制轴：左摇杆 Y 轴（前推为正）
                'axis_linear': 1,
                # 转向控制轴：右摇杆 X 轴（左推为正）
                'axis_steer': 2,

                # ── 按键编号（北通 BFM 模式实测）──────────────
                # 使能按键：RB 键（Button 7），按住才能控制底盘
                'enable_button': 7,
                # 紧急停止按键：B 键（Button 1），按下立即 stop
                'stop_button': 1,
                # 升降上升按键：Y 键（Button 4），长按持续发送 up
                'button_lift_up': 4,
                # 升降下降按键：A 键（Button 0），长按持续发送 down
                'button_lift_down': 0,

                # ── 其他选项 ──────────────────────────────────
                # 转向方向取反（若实际转向与期望相反，改为 True）
                'steer_invert': False,
                # 控制定时器频率（Hz），决定串口指令下发速率
                'publish_rate': 20.0,
            }],
            output='screen',
        ),
    ])
