#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter')
        
        # 默认值与 diuniu_nav.launch.py / diuniu_mapping.launch.py 保持一致：
        # 过滤盒必须覆盖全局 footprint（x∈[-0.30, 1.60]）+ 尾部货叉/托盘载货区
        # （货叉在后轮侧，叉尖/托盘尾端约 x=-1.45m；切片降到地面 0.20m 后载货进入扫描层）
        self.declare_parameter('x_min', -1.65)
        self.declare_parameter('x_max', 1.60)
        self.declare_parameter('y_min', -0.45)
        self.declare_parameter('y_max', 0.45)
        self.declare_parameter('laser_x_offset', 0.0)  # /scan 已在 base_link 系投影，无需雷达偏移
        self.declare_parameter('laser_y_offset', 0.0)
        
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.laser_x_offset = self.get_parameter('laser_x_offset').value
        self.laser_y_offset = self.get_parameter('laser_y_offset').value
        
        # Configure Best Effort QoS with low queue depth to prevent buffer bloat
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile)
        
        self.get_logger().info(f"laserscan_filter started. Filtering box in base_link: x=[{self.x_min}, {self.x_max}], y=[{self.y_min}, {self.y_max}]")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        
        for i in range(len(ranges)):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            
            # LaserScan 极坐标 (r, angle) 原点始终位于物理雷达硬件中心 (laser_link, X=+1.215m)
            # 必须加上 laser_x_offset 才能精准还原到车体中心 base_link 坐标系！
            x_base = x_laser + self.laser_x_offset
            y_base = y_laser + self.laser_y_offset
            
            # Check if point falls inside the filtered bounding box (robot footprint region)
            if self.x_min <= x_base <= self.x_max and self.y_min <= y_base <= self.y_max:
                ranges[i] = float('inf')  # Filter out by setting to infinity
                
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
