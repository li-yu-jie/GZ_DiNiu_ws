#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AprilTag 实时距离监视器 —— 终端滚动刷新显示 tag0 的距离/偏移/姿态"""
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException

# 偏航定义与 tag_align 节点同源（rootfs 无 tf_transformations，统一收归 tf_utils）；
# sys.path 兜底让脚本不 source install 也能直接 python3 运行
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from diuniu_apriltag.tf_utils import quat_yaw


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
            yaw = math.degrees(quat_yaw(q.w, q.x, q.y, q.z))
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
