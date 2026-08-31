#!/usr/bin/env python3
"""
分析并深度清洗 map.pgm 静态地图中的建图残留噪点。

根因分析：
  建图(FAST-LIO + pcd2pgm)时，3D雷达打在地牛车身/货叉/地面的自遮挡点
  被写入 scans.pcd → 经 pcd2pgm z切片投影 → 变成 map.pgm 走廊中央的黑色像素。
  nav2 static_layer 加载后在其周围膨胀出青色/紫色代价圈，堵住走廊。

修复策略：
  1. 形态学开运算(morphological opening)：用 3x3 结构元素做开运算，
     消除小于 3×3 的孤立黑斑和细线弧段，保留墙体等连续实体。
  2. 连通域面积过滤：删除面积 < threshold 像素的黑色连通域。

安全约定（★2026-08-28 起强制执行）：
  - 覆盖写 map.pgm 之前，先把原图备份为同目录
    map_before_clean_YYYYMMDD_HHMMSS.pgm——commit 9d08f3a 已经 revert 过
    一次"过度清洗导致墙体丢失"的地图，没有备份的清洗脚本就是在赌。
  - 地图路径一律从 src/diuniu_nav/maps/map.yaml 现读（common.load_map_meta），
    不再硬编码绝对路径（旧版烤死 /home/y/GZ_DiNiu_ws/...，在 Docker rootfs
    真实路径下直接打不开文件）。

用法：
  python3 tools/clean_map.py                 # 默认 MIN_AREA=15
  python3 tools/clean_map.py --min-area 30   # 更激进的面积阈值
"""
import argparse
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_map_meta  # noqa: E402

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from scipy import ndimage  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='map.pgm 噪点清洗（先备份再覆盖）')
    ap.add_argument('--min-area', type=int, default=15,
                    help='小于该面积的黑色连通域视为噪点擦除（默认 15 像素）')
    args = ap.parse_args()

    map_path, _origin, res = load_map_meta()
    arr = np.array(Image.open(map_path))

    print(f"地图: {map_path} (resolution={res} m/px)")
    print(f"地图尺寸: {arr.shape}")
    print(f"修复前 - 黑色像素(val<100): {np.sum(arr < 100)}")

    # ---- 策略1：形态学开运算（erosion+dilation）清除小于3px的孤立噪点 ----
    binary_obstacle = arr < 100
    kernel = np.ones((3, 3), dtype=bool)
    opened = ndimage.binary_opening(binary_obstacle, structure=kernel)

    # ---- 策略2：连通域面积过滤，删除面积 < min_area 像素的小连通域 ----
    labeled, num_features = ndimage.label(opened)
    print(f"检测到 {num_features} 个独立障碍物连通域")

    # np.bincount 一次算出所有连通域面积（旧版逐 label np.sum(labeled==id)
    # 是 O(N域 × 全图像素)，几千个连通域要扫几千遍全图）
    areas = np.bincount(labeled.ravel())
    areas[0] = 0  # 背景不算
    noise_labels = np.flatnonzero(areas < args.min_area)
    noise_mask = np.isin(labeled, noise_labels)

    cleaned = np.copy(arr)
    cleaned[noise_mask] = 254

    # 也要擦除开运算就移除掉的像素（原来是黑色，开运算后变白）
    erased_by_opening = binary_obstacle & (~opened)
    cleaned[erased_by_opening] = 254

    print(f"形态学开运算消除的噪点像素: {np.sum(erased_by_opening)}")
    print(f"连通域面积过滤消除的噪点域: {len(noise_labels)} 个")
    print(f"修复后 - 黑色像素(val<100): {np.sum(cleaned < 100)}")
    print(f"总计消除噪点像素: {np.sum(arr < 100) - np.sum(cleaned < 100)}")

    # ★ 先备份再覆盖（清洗不可逆，墙体被误擦只能靠自己留的底）
    stamp = time.strftime('%Y%m%d_%H%M%S')
    backup = os.path.join(os.path.dirname(map_path),
                          f'map_before_clean_{stamp}.pgm')
    shutil.copy2(map_path, backup)
    print(f"已备份原图: {backup}")

    Image.fromarray(cleaned).save(map_path)
    print(f"已保存清洗后的地图至 {map_path}")


if __name__ == '__main__':
    main()
