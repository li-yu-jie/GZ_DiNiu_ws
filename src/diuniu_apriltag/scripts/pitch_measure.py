#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机俯仰角反测工具 —— 用 AprilTag 实测镜头光轴的俯仰角

原理：把码竖直贴墙、码中心与镜头中心严格同高（连线即水平线），
则码在相机坐标系里的仰角 = 相机的俯仰角。
camera_optical_frame 约定：z 前、x 右、y 下 → pitch = atan2(-y, z)

用法：
  1. 码竖直贴墙（水平仪靠直），码中心离地高度 = 镜头离地高度
  2. 启动识别链路（start_apriltag.sh），车正对码 ~1m
  3. 运行本脚本，扶稳读数；同时按"水平对中"提示把码挪到画面正中
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException


def main():
    rclpy.init()
    n = Node('camera_pitch_measure')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    last_print = 0.0
    missing_since = None
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.time()
        if now - last_print < 0.1:      # 10Hz 刷新
            continue
        last_print = now
        try:
            t = buf.lookup_transform('camera_optical_frame', 'tag0', Time())
            tr = t.transform.translation
            pitch = math.degrees(math.atan2(-tr.y, tr.z))   # 相机俯仰角(低头+)
            center = math.degrees(math.atan2(tr.x, tr.z))   # 码偏离画面中心的水平角
            line = (f"\r俯仰角 pitch: {pitch:+6.2f}°   距离: {tr.z*100:5.1f} cm   "
                    f"水平对中: {center:+5.1f}° (挪到≈0 再读数)    ")
            print(line, end='', flush=True)
            missing_since = None
        except TransformException:
            if missing_since is None:
                missing_since = now
            print(f"\r未检测到码…（已丢失 {now - missing_since:4.1f}s）"
                  f"{' ' * 40}", end='', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
