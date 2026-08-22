#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AprilTag 实时距离监视器 —— 终端滚动刷新显示 tag0 的距离/偏移/姿态"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException


def main():
    rclpy.init()
    n = Node('tag_distance_watch')
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
            q = t.transform.rotation
            # 四元数 → 绕竖直轴的偏航角（码正对自己时 ~0）
            yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            line = (f"\r距离 z: {tr.z*100:6.1f} cm   左右 x: {tr.x*100:+6.1f} cm   "
                    f"上下 y: {tr.y*100:+6.1f} cm   偏航: {yaw:+6.1f}°    ")
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
