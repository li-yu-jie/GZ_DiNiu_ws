#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼底盘死算里程计 (Ackermann Odometry Dead Reckoning)
===================================================
核心推算逻辑：
  - 编码器 (Encoder): 提供线速度 vx 或单步位移 ds (m)
  - IMU (BNO085): 直接提供车身绝对航向角 Yaw (消除纯编码器积分的角度漂移)
  - 算法: 中点弧线积分法 (Midpoint Runge-Kutta 2nd Order Integration)
"""

import math
import threading
from typing import Tuple, Dict, Any, Optional
from Path.location_table import normalize_angle_deg, normalize_angle_rad


class AckermannOdometry:
    def __init__(self, init_x: float = 0.0, init_y: float = 0.0, init_theta_deg: float = 0.0, scale_factor: float = 1.0):
        """
        初始化里程计推算器
        
        :param init_x: 初始 X 坐标 (m)
        :param init_y: 初始 Y 坐标 (m)
        :param init_theta_deg: 初始航向角 (度)
        :param scale_factor: 编码器线速度/位移标定系数 (默认 1.0)
        """
        self._lock = threading.Lock()
        
        # 世界坐标系绝对位姿
        self.x = float(init_x)
        self.y = float(init_y)
        self.theta_rad = math.radians(init_theta_deg)
        
        # 标定与统计参数
        self.scale_factor = scale_factor
        self.total_distance = 0.0
        
        # IMU 零位偏置与初始状态记录
        self.raw_imu_yaw_rad: Optional[float] = None
        self.imu_offset_rad = 0.0
        self.has_first_imu = False
        
        # 调试/更新统计
        self.update_count = 0

    def set_imu_yaw(self, raw_yaw_deg: float):
        """
        更新绝对 IMU Yaw 角度 (度)
        若首次接收，自动绑定当前绝对位姿的偏置。
        """
        raw_yaw_rad = math.radians(raw_yaw_deg)
        with self._lock:
            if not self.has_first_imu:
                # 记录开机/初次零位映射: world_theta = raw_imu - offset  => offset = raw_imu - world_theta
                self.imu_offset_rad = raw_yaw_rad - self.theta_rad
                self.has_first_imu = True
            
            self.raw_imu_yaw_rad = raw_yaw_rad
            # 更新当前世界航向角
            self.theta_rad = normalize_angle_rad(raw_yaw_rad - self.imu_offset_rad)

    def set_imu_quaternion(self, qw: float, qx: float, qy: float, qz: float):
        """利用四元数计算 Yaw 角度并更新"""
        # 四元数转 Yaw: atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)
        self.set_imu_yaw(yaw_deg)

    def update(self, vx: float, dt: float, raw_imu_yaw_deg: Optional[float] = None) -> Tuple[float, float, float]:
        """
        根据当前帧线速度 vx (m/s) 和时间间隔 dt (s) 进行死算推演
        
        :param vx: 底盘上报的线速度 (m/s)
        :param dt: 采样间隔时间 (s)
        :param raw_imu_yaw_deg: 可选的最新 IMU Yaw 角度 (度)
        :return: 最新位姿 (x, y, theta_deg)
        """
        if dt <= 0.0 or dt > 1.0:
            return self.get_pose()

        ds = vx * self.scale_factor * dt
        return self.update_step(ds, raw_imu_yaw_deg)

    def update_step(self, ds: float, raw_imu_yaw_deg: Optional[float] = None) -> Tuple[float, float, float]:
        """
        根据单步线位移 ds (m) 及可选 IMU Yaw 进行坐标推算 (中点积分)
        
        :param ds: 单步物理位移 (m)
        :param raw_imu_yaw_deg: 最新 IMU Yaw (度)
        :return: 最新位姿 (x, y, theta_deg)
        """
        with self._lock:
            prev_theta_rad = self.theta_rad
            
            # 如果传入了新的 IMU 角度，先更新当前航向
            if raw_imu_yaw_deg is not None:
                raw_yaw_rad = math.radians(raw_imu_yaw_deg)
                if not self.has_first_imu:
                    self.imu_offset_rad = raw_yaw_rad - self.theta_rad
                    self.has_first_imu = True
                self.raw_imu_yaw_rad = raw_yaw_rad
                self.theta_rad = normalize_angle_rad(raw_yaw_rad - self.imu_offset_rad)
            
            curr_theta_rad = self.theta_rad
            
            # 计算中点角度 (Midpoint Runge-Kutta 2nd order)
            delta_theta = normalize_angle_rad(curr_theta_rad - prev_theta_rad)
            mid_theta_rad = prev_theta_rad + 0.5 * delta_theta
            
            # 位置积分
            dx = ds * math.cos(mid_theta_rad)
            dy = ds * math.sin(mid_theta_rad)
            
            self.x += dx
            self.y += dy
            self.total_distance += abs(ds)
            self.update_count += 1

            return self.x, self.y, math.degrees(self.theta_rad)

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta_deg: float = 0.0):
        """
        重置绝对坐标与航向角 (例如到达待机位或完成视觉对齐后校准)
        """
        with self._lock:
            self.x = float(x)
            self.y = float(y)
            self.theta_rad = math.radians(theta_deg)
            if self.raw_imu_yaw_rad is not None:
                self.imu_offset_rad = self.raw_imu_yaw_rad - self.theta_rad

    def reset_distance(self):
        """重置累计里程统计"""
        with self._lock:
            self.total_distance = 0.0

    def get_pose(self) -> Tuple[float, float, float]:
        """获取当前绝对坐标及航向角 (x_m, y_m, theta_deg)"""
        with self._lock:
            return self.x, self.y, normalize_angle_deg(math.degrees(self.theta_rad))

    def get_pose_rad(self) -> Tuple[float, float, float]:
        """获取当前绝对坐标及航向角 (x_m, y_m, theta_rad)"""
        with self._lock:
            return self.x, self.y, self.theta_rad

    def get_summary(self) -> Dict[str, Any]:
        """获取状态摘要字典"""
        with self._lock:
            return {
                "x": round(self.x, 4),
                "y": round(self.y, 4),
                "theta_deg": round(normalize_angle_deg(math.degrees(self.theta_rad)), 2),
                "total_distance": round(self.total_distance, 3),
                "update_count": self.update_count,
                "has_imu": self.has_first_imu
            }
