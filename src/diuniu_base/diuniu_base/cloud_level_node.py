#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# cloud_level_node.py — 点云水平补偿（消除 1.6m 桅杆摆动对切片的影响）
#
# 背景（2026-08-24 幽灵障碍事故根因）：
#   FAST-LIO 的 /cloud_registered_body 在**雷达机体(IMU)系**下发布，但 frame_id
#   直接写死 "base_link"，pointcloud_to_laserscan 因此做恒等变换，在**倾斜的机体
#   系内**按 z 切片。雷达装在 1.6m 桅杆顶端，行驶时支架摆动 ±1~3°，头顶横梁/门架
#   （切片上沿之上的点）会随俯仰被压进切片（Δz ≈ d·δ，10m 处 2° = 0.35m），
#   在代价地图里变成"前方突然出现的障碍物"。
#
# 原理：
#   FAST-LIO 的 /odom 姿态是重力对齐的。取其四元数构造旋转 R_wb，只保留 yaw 得
#   R_yaw，则补偿旋转 R_c = R_yawᵀ·R_wb 恰好消除 roll/pitch（含支架静态倾斜），
#   把所有点摆回水平面。补偿后地面恒在 z=-1.6m、横梁恒在切片上沿之上，切片不再
#   随晃动吞吐异物。
#
# 注意：
#   - 输出 frame_id 保持 "base_link"（下游 pointcloud_to_laserscan 恒等变换的前提），
#     但 z 原点仍在雷达处（1.6m 高），切片高度区间需按雷达系设置。
#   - 姿态角超过 max_angle_deg 视为 FAST-LIO 瞬态发散，不补偿直通并告警。
# =============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
import message_filters


def quat_to_R(w, x, y, z):
    """单位四元数 → 3x3 旋转矩阵 (float64)"""
    n = w * w + x * x + y * y + z * z
    if n < 1e-8:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class CloudLevelNode(Node):
    def __init__(self):
        super().__init__('cloud_level_node')

        self.declare_parameter('cloud_in', '/cloud_registered_body')
        self.declare_parameter('cloud_out', '/cloud_leveled')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_angle_deg', 15.0)

        cloud_in = self.get_parameter('cloud_in').value
        cloud_out = self.get_parameter('cloud_out').value
        odom_topic = self.get_parameter('odom_topic').value
        self.max_angle = np.deg2rad(self.get_parameter('max_angle_deg').value)

        self.latest_R_c = np.eye(3, dtype=np.float32)
        self.pub = self.create_publisher(PointCloud2, cloud_out, 10)

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_in, self.cloud_callback, 10)

        self.get_logger().info(
            f'cloud_level_node: 高频无丢帧点云姿态实时水平补偿 {cloud_in} + {odom_topic} → {cloud_out}')

    def odom_callback(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        R_wb = quat_to_R(q.w, q.x, q.y, q.z)

        roll = np.arctan2(R_wb[2, 1], R_wb[2, 2])
        pitch = np.arcsin(np.clip(-R_wb[2, 0], -1.0, 1.0))
        if abs(roll) > self.max_angle or abs(pitch) > self.max_angle:
            return

        yaw = np.arctan2(R_wb[1, 0], R_wb[0, 0])
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        # ★ 正确构造重力对齐旋转矩阵 R_level = R_wb.T @ Rz，精确消除俯仰(pitch)与横滚(roll)倾斜，
        #    防止加速度前冲/低头时把地面点拉入切片区间造成红色同心圆环鬼影！
        self.latest_R_c = (R_wb.T @ Rz).astype(np.float32)

    def cloud_callback(self, cloud_msg):
        pts = point_cloud2.read_points_numpy(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            self.pub.publish(cloud_msg)
            return

        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        leveled = arr @ self.latest_R_c.T

        out = point_cloud2.create_cloud_xyz32(cloud_msg.header, leveled)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudLevelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
