#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AGV 载货/空载 轮廓动态切换脚本
==================================
⚠️ 2026-08-28 起基本废弃：整车几何定案为"货叉从车尾向前 1.20m 伸在车身范围内、
车尾无伸出"（用户实车确认），整车轮廓即车身轮廓 [-0.30,+1.60]×±0.35；
载货按车身长宽计（托盘规格不一，不留载货余量）。nav2_params.yaml 静态 footprint
已覆盖（全局 [+1.65/-0.35]×±0.40、局部 [+1.60/-0.30]×±0.35），不再按载货状态切换。
本脚本仅保留为"误改轮廓后恢复定案值"的手动工具。

⚠️ 两个历史错误均勿回退：①"叉在车头"（前 1.90/后 -0.30）；②"叉尖伸出车尾 -1.65"
（footprint 拖 1.35m 幻影尾巴，RViz 大绿框）。

使用方法：
  ros2 run diuniu_nav agv_cargo_switcher --ros-args -p mode:=loaded   # 恢复定案轮廓
  ros2 run diuniu_nav agv_cargo_switcher --ros-args -p mode:=empty    # 同上（两模式已一致）
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

        # 2026-08-28 定案几何（与 nav2_params.yaml 静态全局 footprint 一致）：
        # 前 +1.65m（车头 1.60+5cm）/ 后 -0.35m（车尾 -0.30+5cm，车尾无伸出）/ 半宽 0.40m
        # 载货/空载两模式一致：载货按车身长宽计
        coords = [[1.65, 0.40], [1.65, -0.40], [-0.35, -0.40], [-0.35, 0.40]]
        if mode == 'loaded':
            self.get_logger().info("📦 [载货模式] 恢复定案轮廓 (前+1.65/后-0.35/宽±0.40，与静态配置一致)...")
        else:
            self.get_logger().info("🚜 [空载模式] 恢复定案轮廓 (与载货一致：整车轮廓即车身轮廓)...")

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
