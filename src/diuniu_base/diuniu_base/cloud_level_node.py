#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# cloud_level_node.py — 点云水平补偿 + 坐标系诚实化（雷达系 → 真实 base_link）
#
# 背景（2026-08-24 幽灵障碍事故根因）：
#   FAST-LIO 的 /cloud_registered_body 在**雷达机体(IMU)系**下发布，但 frame_id
#   直接写死 "base_link"，pointcloud_to_laserscan 因此做恒等变换，在**倾斜的机体
#   系内**按 z 切片。雷达支架摆动 ±1~3° 时，切片上沿之上的结构点会随俯仰被压进
#   切片（Δz ≈ d·δ，10m 处 2° = 0.35m），在代价地图里变成"前方突然出现的障碍物"。
#
# 原理：
#   FAST-LIO 的 /odom 姿态是重力对齐的。取其四元数构造旋转 R_wb，只保留 yaw 得
#   R_yaw，则补偿旋转 R_c = R_yawᵀ·R_wb 恰好消除 roll/pitch（含支架静态倾斜），
#   把所有点摆回水平面，切片不再随晃动吞吐异物。
#
# 坐标系诚实化（2026-08-28）：
#   ★ 本节点是安全链上唯一逐点处理点云的节点，在此把 FAST-LIO 的"坐标谎言"
#     一并修正：输出点 = R_c·p + t_lidar，其中 t_lidar=(1.215, 0, 0.66) 是雷达
#     在 base_link 中的安装位置（须与 URDF laser_joint 一致，可用参数覆盖）。
#     输出 frame_id 仍为 "base_link"，但从此**名副其实**：
#       - z=0 在地面，切片高度按地面系设置（pointcloud_to_laserscan
#         min_height=0.20 / max_height=1.20）；
#       - x/y 原点在 base_link（后轴中心），laserscan_filter 屏蔽盒、
#         collision_monitor 多边形、costmap footprint 的全部 base_link 数值
#         均为真实几何，不再需要任何"雷达系"心算换算。
#   ⚠️ 雷达若再次移位，必须同步改 base_offset_x/y/z（或 URDF），否则全链平移错位。
#
# 失败语义（fail-open，不补偿直通）：
#   - 未收到 odom / odom 姿态超 max_angle_deg / odom 超时（>0.3s）：
#     不做旋转补偿（按水平处理），但**照常发布**平移后的点云并节流告警。
#     绝不丢帧——本节点丢帧等于 /scan 全链断流、代价地图与 collision_monitor
#     集体失明（车在动却没有障碍数据，比不补偿更危险）。
# =============================================================================

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry


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
        # 雷达在 base_link 中的安装位置 (m)，须与 URDF laser_joint 一致
        self.declare_parameter('base_offset_x', 1.215)
        self.declare_parameter('base_offset_y', 0.0)
        self.declare_parameter('base_offset_z', 0.66)

        cloud_in = self.get_parameter('cloud_in').value
        cloud_out = self.get_parameter('cloud_out').value
        odom_topic = self.get_parameter('odom_topic').value
        self.max_angle = np.deg2rad(self.get_parameter('max_angle_deg').value)
        gp = self.get_parameter
        self.offset = np.array([gp('base_offset_x').value,
                                gp('base_offset_y').value,
                                gp('base_offset_z').value], dtype=np.float32)

        self.latest_R_c = np.eye(3, dtype=np.float32)
        self.leveled_ok = False        # 当前是否持有有效的补偿旋转
        self.latest_odom_time = None
        self.odom_count = 0
        self.pub = self.create_publisher(PointCloud2, cloud_out, 10)

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_in, self.cloud_callback, 10)

        self.get_logger().info(
            f'cloud_level_node: 点云水平补偿+坐标系诚实化 {cloud_in} + {odom_topic} → {cloud_out} '
            f'(base_offset={[float(v) for v in self.offset]})')

    def odom_callback(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        R_wb = quat_to_R(q.w, q.x, q.y, q.z)

        roll = np.arctan2(R_wb[2, 1], R_wb[2, 2])
        pitch = np.arcsin(np.clip(-R_wb[2, 0], -1.0, 1.0))
        if abs(roll) > self.max_angle or abs(pitch) > self.max_angle:
            # FAST-LIO 瞬态发散：本帧不更新补偿角（保留上一有效值），不冻结时间戳
            self.get_logger().warning(
                f'cloud_level_node: odom orientation exceeds max angle! roll: {np.rad2deg(roll):.2f}°, pitch: {np.rad2deg(pitch):.2f}°',
                throttle_duration_sec=2.0
            )
            return

        yaw = np.arctan2(R_wb[1, 0], R_wb[0, 0])
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        # ★ 正确构造重力对齐旋转矩阵 R_level = R_wb.T @ Rz，精确消除俯仰(pitch)与横滚(roll)倾斜，
        #    防止加速度前冲/低头时把地面点拉入切片区间造成红色同心圆环鬼影！
        self.latest_R_c = (R_wb.T @ Rz).astype(np.float32)
        self.leveled_ok = True
        self.latest_odom_time = odom_msg.header.stamp

        # 每 100 帧 odom 打印一次补偿角调试日志
        self.odom_count += 1
        if self.odom_count % 100 == 0:
            self.get_logger().info(
                f'cloud_level_node status: roll = {np.rad2deg(roll):.2f}°, pitch = {np.rad2deg(pitch):.2f}°'
            )

    def _rotation_usable(self, cloud_msg):
        """判定本次发布能否使用补偿旋转；不可用则 fail-open 按水平处理。"""
        if not self.leveled_ok:
            self.get_logger().warning(
                'cloud_level_node: 尚未收到有效 odom，按水平直通发布（不做旋转补偿）。',
                throttle_duration_sec=2.0
            )
            return False
        cloud_sec = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
        odom_sec = self.latest_odom_time.sec + self.latest_odom_time.nanosec * 1e-9
        time_diff = abs(cloud_sec - odom_sec)
        if time_diff > 0.3:
            self.get_logger().warning(
                f'cloud_level_node: odom 过期 (time diff: {time_diff:.3f}s > 0.3s)，'
                f'本帧按水平直通发布（不做旋转补偿）。',
                throttle_duration_sec=2.0
            )
            return False
        return True

    def cloud_callback(self, cloud_msg):
        pts = point_cloud2.read_points_numpy(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            self.pub.publish(cloud_msg)
            return

        arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        if self._rotation_usable(cloud_msg):
            arr = arr @ self.latest_R_c.T
        # 雷达系 → 真实 base_link：z=0 落到地面，x/y 原点后移到后轴中心
        out = point_cloud2.create_cloud_xyz32(cloud_msg.header, arr + self.offset)
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
