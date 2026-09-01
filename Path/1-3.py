#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
# 自动将工作区根目录加入 sys.path，支持在任意目录下运行脚本
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from Path.location_table import save_coordinate_to_file, get_location
except ModuleNotFoundError:
    from location_table import save_coordinate_to_file, get_location

# 1. 保存/更新一个新坐标 "point_7"
save_coordinate_to_file(point_name="a", x=1.0, y=1.0, theta_deg=0.0)

# 2. 读取点名坐标数据
pos = get_location("a")
print(f"✅ 读取坐标结果: a = {pos}")