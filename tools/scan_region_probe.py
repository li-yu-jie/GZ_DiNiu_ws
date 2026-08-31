#!/usr/bin/env python3
# scan_region_probe.py — 统计 /scan 在指定基座系区域内的点数与帧持续率
# 用途：雷达移回 0.66m 正装后，验证屏蔽盒解封区域是否还有鬼影
# 屏蔽盒（须与 laserscan_filter 参数/两个 launch 同步）：x∈[-1.65,1.65], y=±0.36
# 区域：A=盒前缘外 [1.65,2.60]×±0.36  B=盒左缘外 x[-1.65,1.65]×y[0.36,1.60]  C=右侧镜像
import math
import rclpy
from rclpy.node import Node
from common import scan_xy, subscribe_scan

# 屏蔽盒边界（须与 laserscan_filter 参数/两个 launch 同步）
BOX_X0, BOX_X1, BOX_Y = -1.65, 1.65, 0.36
REGIONS = {
    'A_front': (BOX_X1, 2.60, -BOX_Y, BOX_Y),
    'B_left':  (BOX_X0, BOX_X1, BOX_Y, 1.60),
    'C_right': (BOX_X0, BOX_X1, -1.60, -BOX_Y),
}
DUR = 6.0

class RegionProbe(Node):
    def __init__(self):
        super().__init__('scan_region_probe')
        self.frames = 0
        self.hits = {k: 0 for k in REGIONS}      # 有点的帧数
        self.total = {k: 0 for k in REGIONS}     # 总点数
        self.min_r = {k: 99.0 for k in REGIONS}  # 最近点距离
        subscribe_scan(self, '/scan', self.cb)
        self.t0 = self.get_clock().now()

    def cb(self, msg):
        if (self.get_clock().now() - self.t0).nanoseconds / 1e9 > DUR:
            return
        self.frames += 1
        frame_hits = {k: 0 for k in REGIONS}
        for x, y in scan_xy(msg):
            r = math.hypot(x, y)
            if msg.range_min <= r <= msg.range_max:
                for k, (x0, x1, y0, y1) in REGIONS.items():
                    if x0 <= x <= x1 and y0 <= y <= y1:
                        frame_hits[k] += 1
                        if r < self.min_r[k]:
                            self.min_r[k] = r
        for k in REGIONS:
            if frame_hits[k] > 0:
                self.hits[k] += 1
                self.total[k] += frame_hits[k]

    def report(self):
        print(f'采样 {self.frames} 帧 /scan ({DUR}s)')
        for k in REGIONS:
            pct = 100.0 * self.hits[k] / max(self.frames, 1)
            avg = self.total[k] / max(self.hits[k], 1)
            print(f'{k}: 有点帧率 {pct:5.1f}%  平均点数 {avg:5.1f}  最近距离 {self.min_r[k]:.2f}m')

rclpy.init()
node = RegionProbe()
import time
end = time.time() + DUR + 1.0
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.report()
