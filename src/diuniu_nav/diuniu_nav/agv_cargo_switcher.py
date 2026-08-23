#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGV 载货/空载 轮廓动态切换脚本
==================================
使用方法：
  ros2 run diuniu_nav agv_cargo_switcher --ros-args -p mode:=loaded   # 切换为载货模式 (扩大轮廓 2.40m x 0.90m)
  ros2 run diuniu_nav agv_cargo_switcher --ros-args -p mode:=empty    # 切换为空载模式 (恢复车身轮廓 1.90m x 0.70m)
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32

class AGVCargoSwitcher(Node):
    def __init__(self):
        super().__init__('agv_cargo_switcher')
        
        self.declare_parameter('mode', 'loaded')
        mode = self.get_parameter('mode').value.lower()
        
        self.pub_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.pub_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)
        
        poly = Polygon()
        
        if mode == 'loaded':
            self.get_logger().info("📦 正在切换至 [载货模式] —— 扩大代价地图避障轮廓 (长 2.4m x 宽 0.9m)...")
            # 载货轮廓：前 1.90m, 后 -0.50m, 宽 +-0.45m
            coords = [[1.90, 0.45], [1.90, -0.45], [-0.50, -0.45], [-0.50, 0.45]]
        else:
            self.get_logger().info("🚜 正在切换至 [空载模式] —— 恢复车身标准轮廓 (长 1.9m x 宽 0.7m)...")
            # 标准空载轮廓：前 1.60m, 后 -0.30m, 宽 +-0.35m
            coords = [[1.60, 0.35], [1.60, -0.35], [-0.30, -0.35], [-0.30, 0.35]]
            
        for pt in coords:
            p = Point32()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.0
            poly.points.append(p)
            
        # 连续发送 3 次确保广播到位
        for _ in range(3):
            self.pub_global.publish(poly)
            self.pub_local.publish(poly)
            
        self.get_logger().info("✅ AGV 避障轮廓已成功更新至 Nav2 Costmap！")

def main(args=None):
    rclpy.init(args=args)
    node = AGVCargoSwitcher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
