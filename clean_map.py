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
"""
from PIL import Image
import numpy as np

map_path = "/home/y/GZ_DiNiu_ws/src/diuniu_nav/maps/map.pgm"
img = Image.open(map_path)
arr = np.array(img)

print(f"地图尺寸: {arr.shape}")
print(f"修复前 - 黑色像素(val<100): {np.sum(arr < 100)}")

# ---- 策略1：形态学开运算（erosion+dilation）清除小于3px的孤立噪点 ----
from scipy import ndimage

# 创建二值掩码：黑色像素=True，白色=False
binary_obstacle = arr < 100

# 开运算：先腐蚀再膨胀，kernel=3x3，消除 <3px 的孤立噪点和弧线
kernel = np.ones((3, 3), dtype=bool)
opened = ndimage.binary_opening(binary_obstacle, structure=kernel)

# ---- 策略2：连通域面积过滤，删除面积 < 15 像素的小连通域 ----
labeled, num_features = ndimage.label(opened)
print(f"检测到 {num_features} 个独立障碍物连通域")

MIN_AREA = 15  # 小于15像素的连通域视为噪点
cleaned = np.copy(arr)
noise_removed = 0
for label_id in range(1, num_features + 1):
    area = np.sum(labeled == label_id)
    if area < MIN_AREA:
        # 这个连通域太小，是噪点，擦除
        cleaned[labeled == label_id] = 254
        noise_removed += 1
    # else: 保留（是真实墙体）

# 也要擦除开运算就移除掉的像素（原来是黑色，开运算后变白）
erased_by_opening = binary_obstacle & (~opened)
cleaned[erased_by_opening] = 254

print(f"形态学开运算消除的噪点像素: {np.sum(erased_by_opening)}")
print(f"连通域面积过滤消除的噪点域: {noise_removed} 个")
print(f"修复后 - 黑色像素(val<100): {np.sum(cleaned < 100)}")
print(f"总计消除噪点像素: {np.sum(arr < 100) - np.sum(cleaned < 100)}")

Image.fromarray(cleaned).save(map_path)
print(f"已保存清洗后的地图至 {map_path}")
