#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# tf_utils.py — diuniu_apriltag 包内共享小工具
#
# 四元数→yaw 全仓曾手抄 9 份（rootfs 无 tf_transformations），统一收归此处。
# 约定与 distance_watch.py 的偏航读数定义一致（四元数 ZYX 绕相机 z 轴）。
# =============================================================================

import math


def quat_yaw(w, x, y, z):
    """单位四元数 → 绕 z 轴 yaw（ZYX 欧拉，rad）。"""
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))
