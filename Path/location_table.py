#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地牛作业点位数字化坐标表、通道航点路由表与几何工具函数
========================================================
包含:
  1. 固定点位坐标表 (LOCATION_TABLE): 停车待机点 parking 与 1~6 号取放货点
  2. 通道航点坐标表 (WAYPOINT_TABLE): 避免碰撞物理障碍物的公共主干通道/拐角航点
  3. 路由路径规划字典 (ROUTE_TABLE) 与 get_route_path() 工具
"""

import os
import json
import math
from typing import Dict, Tuple, List, Union, Optional

# JSON 坐标存放地文件路径
COORDINATES_JSON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "coordinates.json")

# 固定作业点位坐标表 (X_m, Y_m, Theta_deg)
LOCATION_TABLE: Dict[Union[str, int], Tuple[float, float, float]] = {
    "parking": (0.0, 0.0, 0.0),
    1: (2.5, 1.0, 90.0),   # 1号取货点
    2: (4.0, 1.0, 90.0),   # 2号取货点
    3: (2.5, 6.0, -90.0),  # 3号放货点
    4: (4.0, 6.0, -90.0),  # 4号放货点
    5: (5.5, 6.0, -90.0),  # 5号放货点
    6: (7.0, 6.0, -90.0)   # 6号放货点
}

# 避障主干通道/拐角中间航点表 (X_m, Y_m, Theta_deg)
WAYPOINT_TABLE: Dict[str, Tuple[float, float, float]] = {
    "aisle_entry": (1.0, 0.0, 0.0),
    "pick_aisle": (1.0, 1.0, 90.0),
    "mid_cross": (1.0, 3.5, 90.0),
    "drop_aisle": (1.0, 6.0, -90.0),
}


def load_coordinates_from_file() -> Dict[Union[str, int], Tuple[float, float, float]]:
    """从 Path/coordinates.json 自动加载最新坐标表"""
    if os.path.exists(COORDINATES_JSON_PATH):
        try:
            with open(COORDINATES_JSON_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
                for k, v in data.items():
                    key = int(k) if str(k).isdigit() else k
                    LOCATION_TABLE[key] = (float(v[0]), float(v[1]), float(v[2]))
        except Exception as e:
            print(f"⚠️ 加载 {COORDINATES_JSON_PATH} 异常: {e}")
    return LOCATION_TABLE


def save_coordinate_to_file(point_name: Union[str, int], x: float, y: float, theta_deg: float):
    """保存或更新坐标数据到 Path/coordinates.json"""
    key = str(point_name)
    LOCATION_TABLE[int(key) if key.isdigit() else key] = (float(x), float(y), float(theta_deg))
    
    current_data = {}
    if os.path.exists(COORDINATES_JSON_PATH):
        try:
            with open(COORDINATES_JSON_PATH, 'r', encoding='utf-8') as f:
                current_data = json.load(f)
        except Exception:
            pass
            
    current_data[key] = [round(float(x), 3), round(float(y), 3), round(float(theta_deg), 1)]
    
    with open(COORDINATES_JSON_PATH, 'w', encoding='utf-8') as f:
        json.dump(current_data, f, indent=2, ensure_ascii=False)
    print(f"💾 [坐标保存] 成功保存点名 '{key}' -> ({x}, {y}, {theta_deg}°)")


# 初始化自动加载 JSON 存放地数据
load_coordinates_from_file()


def get_location(point_id: Union[str, int]) -> Optional[Tuple[float, float, float]]:
    """获取指定点位编号的绝对坐标 (x, y, theta_deg)"""
    load_coordinates_from_file()  # 保证实时读取最新保存的坐标数据
    if point_id in LOCATION_TABLE:
        return LOCATION_TABLE[point_id]
    try:
        int_id = int(point_id)
        if int_id in LOCATION_TABLE:
            return LOCATION_TABLE[int_id]
    except (ValueError, TypeError):
        pass
    return None


def get_route_path(start_id: Union[str, int], target_id: Union[str, int]) -> List[Tuple[float, float, float]]:
    """
    查表获取从 start_id 到 target_id 的安全避障折线航点列表
    若 ROUTE_TABLE 中未显式配置，则自动生成基于主通道 (X=1.0) 的安全两段折线
    """
    # 转换为标准 key
    start_key = int(start_id) if str(start_id).isdigit() else start_id
    target_key = int(target_id) if str(target_id).isdigit() else target_id
    
    # 1. 优先在固定路由表中匹配
    if (start_key, target_key) in ROUTE_TABLE:
        return ROUTE_TABLE[(start_key, target_key)]
    
    # 2. 自动两段折线路由规则 (直角绕行)
    start_loc = get_location(start_key)
    target_loc = get_location(target_key)
    
    if not start_loc or not target_loc:
        return [target_loc] if target_loc else []
    
    # 若起点与终点 Y 坐标不同，先沿通道移动 Y，再驶向 X 目标
    via_point1 = (start_loc[0], target_loc[1], target_loc[2])
    return [via_point1, target_loc]


def normalize_angle_deg(angle_deg: float) -> float:
    """将角度规范化至 [-180, 180) 范围内"""
    while angle_deg >= 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def normalize_angle_rad(angle_rad: float) -> float:
    """将弧度规范化至 [-pi, pi) 范围内"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def calc_distance_and_heading(current_x: float, current_y: float, 
                              target_x: float, target_y: float) -> Tuple[float, float, float]:
    """
    计算当前坐标到目标坐标的欧氏距离、目标方位角 (度) 和 (弧度)
    
    :return: (distance, heading_deg, heading_rad)
    """
    dx = target_x - current_x
    dy = target_y - current_y
    dist = math.hypot(dx, dy)
    heading_rad = math.atan2(dy, dx)
    heading_deg = math.degrees(heading_rad)
    return dist, heading_deg, heading_rad
