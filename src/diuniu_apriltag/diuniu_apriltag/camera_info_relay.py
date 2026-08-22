#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CameraInfo 坐标系补齐中继节点
==============================
v4l2_camera 只给图像消息设置 frame_id，/camera_info 的 header.frame_id
为空，导致 apriltag_ros 发布 TF 时父坐标系无效（/tf 为空）。

本节点订阅原始 camera_info，补上 frame_id 后转发：

    /camera_info_raw (v4l2_camera)  →  /camera_info (frame_id 已设置)

由 launch/apriltag.launch.py 自动串接，无需单独启动。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    """为 CameraInfo 消息补齐 frame_id 的中继节点。"""

    def __init__(self):
        super().__init__('camera_info_relay')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.sub = self.create_subscription(
            CameraInfo, '/camera_info_raw', self.callback, 10)
        self.get_logger().info(
            f'camera_info 中继: /camera_info_raw -> /camera_info, '
            f'frame_id={self.frame_id}')

    def callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
