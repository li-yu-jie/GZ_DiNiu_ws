#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag 识别数据流启动文件
============================
数据流闭环：
  usb_cam (XW500U3 1080p, 带标定内参)
      → /image_raw + /camera_info
  image_proc/rectify_node (去畸变)
      → /image_rect
  apriltag_ros (36h11 检测 + 位姿估计)
      → /detections + TF(tag<ID>)

使用方法：
  ros2 launch diuniu_apriltag apriltag.launch.py
  ros2 launch diuniu_apriltag apriltag.launch.py tag_size:=0.045   # 实测黑边边长(米)

查看检测结果：
  ros2 topic echo /detections
  ros2 run tf2_ros tf2_echo camera_optical_frame tag0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_info_url = 'file://' + os.path.join(
        get_package_share_directory('diuniu_description'),
        'config', 'camera', 'xw500u3_1920x1080.yaml')
    apriltag_config = os.path.join(
        get_package_share_directory('diuniu_apriltag'),
        'config', 'apriltag.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video3',
            description='XW500U3 USB 相机的 V4L2 设备节点（重插会漂移，启动前确认）'
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=default_info_url,
            description='相机内参标定文件 URL（XW500U3 @ 1920x1080 专用）'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_optical_frame',
            description='相机光学坐标系名称（图像/camera_info 的 frame_id）'
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.1737',
            description='AprilTag 黑色方块外缘边长 (m)，不含白色留白边。'
                        '2026-08-26 斜距反推有效值 17.37cm（标称 18cm 含白边）。'
                        '注意：tag0 的 TF 尺度以 config/apriltag.yaml 的 tag.sizes 为准'
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='true 时打开 RViz：相机画面 + tag 坐标轴叠加显示'
        ),

        # 1. 相机驱动：发布 /image_raw + /camera_info_raw（含标定内参）
        #    XW500U3 只有 MJPG 格式，v4l2_camera 不支持 MJPG 解码，必须用 usb_cam
        #    camera_info 先改名导出，经中继补 frame_id 后再回到 /camera_info
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 30.0,
                'camera_name': 'xw500u3',
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'frame_id': LaunchConfiguration('camera_frame'),
            }],
            remappings=[
                ('camera_info', '/camera_info_raw'),
            ],
            output='screen',
        ),

        # 2. camera_info 中继：补齐 frame_id（v4l2_camera 不设置，
        #    否则 apriltag_ros 无法发布 TF）
        Node(
            package='diuniu_apriltag',
            executable='camera_info_relay',
            name='camera_info_relay',
            parameters=[{
                'frame_id': LaunchConfiguration('camera_frame'),
            }],
            output='screen',
        ),

        # 3. 去畸变：/image_raw + /camera_info → /image_rect
        Node(
            package='image_proc',
            executable='rectify_node',
            name='image_rectify',
            remappings=[
                ('image', '/image_raw'),
                ('camera_info', '/camera_info'),
                ('image_rect', '/image_rect'),
            ],
            output='screen',
        ),

        # 4. AprilTag 检测：/image_rect + /camera_info → /detections + TF
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            remappings=[
                ('image_rect', '/image_rect'),
                ('camera_info', '/camera_info'),
            ],
            parameters=[
                apriltag_config,
                {'size': ParameterValue(
                    LaunchConfiguration('tag_size'), value_type=float)},
            ],
            output='screen',
        ),

        # 5. 静态 TF：world → camera_optical_frame（单位变换）
        #    否则相机坐标系只在识别到码时才出现在 TF 树里，
        #    RViz Camera 显示会因坐标系缺失报 Error 黑屏（时好时坏的根因）。
        #    装车做完外参后，改由机器人 TF 树提供 base_link→camera_optical_frame。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'world', LaunchConfiguration('camera_frame')],
            output='screen',
        ),

        # 6. RViz 可视化（可选）：相机画面 + tag0 坐标轴叠加
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('diuniu_apriltag'),
                'rviz', 'apriltag.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
