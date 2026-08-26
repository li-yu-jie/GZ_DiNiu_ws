#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机俯仰角反测工具（地面码版）—— 用地面上的 AprilTag 实测镜头俯仰角

原理：码平铺在地面（中心离地≈0），镜头高 h，卷尺量出镜头正下方到码中心
的地面距离 d，则世界里码的俯角 α = atan(h/d)；TF 给出码在相机坐标系里
偏离光轴的俯角 β = atan2(y, z)。相机俯仰角 pitch = α − β（低头为正）。

同时输出"对中角"：码若摆在车身中线上，atan2(x, z) 应≈0，
非零即相机偏航安装误差（可并入外参 yaw 修正）。

用法：
  1. 码平放地面、朝上，摆在车尾相机视野内、车身中线附近
  2. 卷尺量：镜头正下方地面点 → 码中心的距离 d（米）
  3. 启动识别链路（start_apriltag.sh）
  4. python3 pitch_measure.py --dist 1.50 [--height 1.85]
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, required=True,
                    help='镜头正下方地面点到码中心的水平距离 (m)')
    ap.add_argument('--height', type=float, default=1.85,
                    help='镜头离地高度 (m)，默认 1.85')
    args = ap.parse_args()

    alpha = math.atan2(args.height, args.dist)   # 世界里码相对镜头的俯角
    print(f"世界俯角 α = atan({args.height}/{args.dist}) = "
          f"{math.degrees(alpha):.2f}°，等待码检测…")

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
            beta = math.atan2(tr.y, tr.z)          # 码偏离光轴的俯角（相机系）
            pitch = alpha - beta                    # 相机俯仰角，低头为正
            yaw_off = math.degrees(math.atan2(tr.x, tr.z))  # 对中/偏航误差
            line = (f"\r俯仰角 pitch: {math.degrees(pitch):+6.2f}°   "
                    f"对中角: {yaw_off:+5.1f}°   "
                    f"斜距: {math.sqrt(tr.x**2+tr.y**2+tr.z**2)*100:5.1f} cm    ")
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
