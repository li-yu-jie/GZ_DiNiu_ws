#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# laserscan_filter.py — AMR 地牛自遮挡 LaserScan 过滤器
#
# 问题根因：
#   Livox MID-360 装在 1.6m 桅杆顶端，射线斜向下打到地面/车身/货叉/护罩，
#   经 pointcloud_to_laserscan 投影后变成 ~2m 处 ±45°~55° 方向的弧形假障碍。
#
# 过滤策略（双层）：
#   1. 矩形盒过滤：删除落在车体包络矩形内的点
#   2. 圆弧过滤：删除在 ±40°~60° 角度范围内、距离 < max_self_radius 的地面反射弧
#      （矩形盒在对角线方向存在覆盖盲区，圆弧过滤补全）
# =============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter')
        
        # ---- 矩形盒参数（车体包络） ----
        self.declare_parameter('x_min', -1.65)
        self.declare_parameter('x_max', 2.60)
        self.declare_parameter('y_min', -1.60)
        self.declare_parameter('y_max', 1.60)
        self.declare_parameter('laser_x_offset', 0.0)
        self.declare_parameter('laser_y_offset', 0.0)
        
        # ---- 圆弧过滤参数（地面反射弧） ----
        # 地面反射弧出现在 ±40°~60° 方向，距离约 1.87~2.10m
        self.declare_parameter('arc_filter_enabled', True)
        self.declare_parameter('arc_angle_min_deg', 38.0)   # 弧形起始角度（°）
        self.declare_parameter('arc_angle_max_deg', 62.0)   # 弧形结束角度（°）
        self.declare_parameter('arc_max_range', 2.20)       # 弧形最大距离（m）
        
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.laser_x_offset = self.get_parameter('laser_x_offset').value
        self.laser_y_offset = self.get_parameter('laser_y_offset').value
        
        self.arc_filter_enabled = self.get_parameter('arc_filter_enabled').value
        self.arc_angle_min = math.radians(self.get_parameter('arc_angle_min_deg').value)
        self.arc_angle_max = math.radians(self.get_parameter('arc_angle_max_deg').value)
        self.arc_max_range = self.get_parameter('arc_max_range').value
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile)
        
        self.get_logger().info(
            f"laserscan_filter: box=[{self.x_min},{self.x_max}]x[{self.y_min},{self.y_max}], "
            f"arc_filter={'ON' if self.arc_filter_enabled else 'OFF'} "
            f"angle=[{math.degrees(self.arc_angle_min):.0f},{math.degrees(self.arc_angle_max):.0f}]° "
            f"max_r={self.arc_max_range:.2f}m")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        
        for i in range(len(ranges)):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            
            x_base = x_laser + self.laser_x_offset
            y_base = y_laser + self.laser_y_offset
            
            # 策略1：矩形盒过滤（车体包络区域内的点）
            if self.x_min <= x_base <= self.x_max and self.y_min <= y_base <= self.y_max:
                ranges[i] = float('inf')
                continue
            
            # 策略2：圆弧过滤（±40°~60° 方向的地面反射弧）
            if self.arc_filter_enabled:
                abs_angle = abs(angle)
                if self.arc_angle_min <= abs_angle <= self.arc_angle_max and r < self.arc_max_range:
                    ranges[i] = float('inf')
                    continue
                
        msg.ranges = ranges
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

