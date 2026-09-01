#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼底盘点到点“盲走”控制器 (go_to)
=====================================
通过订阅/推演 AckermannOdometry 绝对坐标，向 /cmd_vel 发布 Twist 运动指令
到达目标容差区域 (如 distance < 0.25m) 时停车，为视觉 AprilTag 矫正对齐做好准备。
"""

import math
import time
from typing import Tuple, Optional, Callable

try:
    from Path.location_table import normalize_angle_deg, normalize_angle_rad, calc_distance_and_heading
    from Path.ackermann_odometry import AckermannOdometry
except ModuleNotFoundError:
    from location_table import normalize_angle_deg, normalize_angle_rad, calc_distance_and_heading
    from ackermann_odometry import AckermannOdometry


class PointToPointController:
    def __init__(self, 
                 odometry: AckermannOdometry, 
                 cmd_vel_pub_func: Optional[Callable[[float, float, float], None]] = None,
                 wheelbase: float = 1.30,
                 max_linear_speed: float = 0.5,
                 max_angular_speed: float = 0.8,
                 kp_linear: float = 0.8,
                 kp_angular: float = 1.2,
                 dist_tolerance: float = 0.20,
                 angle_tolerance_deg: float = 5.0):
        """
        初始化点到点控制器
        
        :param odometry: 绝对坐标里程计实例
        :param cmd_vel_pub_func: 发送 /cmd_vel 速度回调函数 func(vx, wz, vz_lift)
        :param wheelbase: 阿克曼底盘轴距 (m)
        :param max_linear_speed: 最大前进线速度 (m/s)
        :param max_angular_speed: 最大转向角速度 (rad/s)
        :param kp_linear: 线速度 P 控制比例增益
        :param kp_angular: 角速度 P 控制比例增益
        :param dist_tolerance: 盲走到达目标距离容差 (m)
        :param angle_tolerance_deg: 盲走到达目标角度容差 (度)
        """
        self.odom = odometry
        self.cmd_vel_pub = cmd_vel_pub_func
        self.wheelbase = wheelbase
        self.max_v = max_linear_speed
        self.max_w = max_angular_speed
        self.kp_v = kp_linear
        self.kp_w = kp_angular
        self.dist_tol = dist_tolerance
        self.angle_tol_deg = angle_tolerance_deg
        
        self.is_active = False
        self.align_start_time = None
        self.is_aligning = False

    def publish_cmd_vel(self, vx: float, wz: float, vz_lift: float = 0.0):
        """下发运动指令"""
        if self.cmd_vel_pub:
            self.cmd_vel_pub(vx, wz, vz_lift)

    def stop(self):
        """停车发零速"""
        self.publish_cmd_vel(0.0, 0.0, 0.0)

    def compute_control(self, target_x: float, target_y: float, target_theta_deg: Optional[float] = None) -> Tuple[float, float, bool]:
        """
        计算单步控制输出 (vx, wz, is_arrived)
        
        :return: (vx, wz, is_arrived)
        """
        curr_x, curr_y, curr_th_deg = self.odom.get_pose()
        curr_th_rad = math.radians(curr_th_deg)
        
        # 1. 计算与目标点的绝对距离和期望向量航向
        dist, target_heading_deg, target_heading_rad = calc_distance_and_heading(curr_x, curr_y, target_x, target_y)
        
        # 2. 判断是否已进入盲走到达容差区域 (到点后，原地自转对正终点角度，然后进入下一个点)
        if dist <= self.dist_tol:
            if target_theta_deg is not None:
                # 记录首次进入角度校准的时间
                if self.align_start_time is None:
                    self.align_start_time = time.time()

                angle_err_deg = normalize_angle_deg(target_theta_deg - curr_th_deg)

                # 安全保护：微调时间超时（最多 8.0s）
                if time.time() - self.align_start_time > 8.0:
                    print(f"\n⚠️ [GO_TO_终点对齐] 原地自转对正微调已超时 (8.0s)，中止调整直接判定到达！")
                    return 0.0, 0.0, True

                if abs(angle_err_deg) <= self.angle_tol_deg:
                    return 0.0, 0.0, True
                else:
                    # 原地打角调整角度 (限制最大原地角速度为 0.45 rad/s)
                    wz_val = self.kp_w * math.radians(angle_err_deg)
                    wz = max(-0.45, min(0.45, wz_val))
                    if abs(wz) < 0.15:
                        wz = math.copysign(0.15, angle_err_deg)
                    return 0.0, wz, False
            else:
                return 0.0, 0.0, True
        
        # 3. 未到达目标：计算航向偏差角
        # 根据前进/倒车确定期望航向
        target_yaw_rad = target_heading_rad if self.max_v >= 0 else normalize_angle_rad(target_heading_rad - math.pi)
        heading_err_rad = normalize_angle_rad(target_yaw_rad - curr_th_rad)
        heading_err_deg = math.degrees(heading_err_rad)
        
        # 4. 原地自正与行驶逻辑 (大偏角原地自转对正 - 迟滞环控制)
        if self.is_aligning:
            if abs(heading_err_deg) < 5.0:
                self.is_aligning = False
                print(f"\n🎉 [GO_TO_原地对正] 航向已对准到 {heading_err_deg:.1f}°，退出自转并切入行驶！")
        else:
            # 只有距离目标大于 0.50 米时，才允许因偏差过大触发起跑原地自转，屏蔽临近点处的航向角奇异性
            if dist > 0.50 and abs(heading_err_deg) > 15.0:
                self.is_aligning = True

        if self.is_aligning:
            vx = 0.0
            wz_val = self.kp_w * heading_err_rad
            wz = max(-0.45, min(0.45, wz_val))
            if abs(wz) < 0.15:
                wz = math.copysign(0.15, heading_err_rad)
            return vx, wz, False
        else:
            # 航向已对齐，直行 (引入 1.5度控制死区，防止前轮高频抖动)
            # 接近目标线性减速，避免恒速冲进容差圈后过冲 → 反复"停-转-走"振荡
            v_mag = min(abs(self.max_v), max(0.10, 1.2 * dist))
            vx = math.copysign(v_mag, self.max_v if self.max_v != 0 else 1.0)
            if abs(heading_err_deg) <= 1.5:
                wz = 0.0
            else:
                wz = self.kp_w * heading_err_rad
                wz = max(-0.20, min(0.20, wz))
            return vx, wz, False

    def go_to(self, target_x: float, target_y: float, target_theta_deg: Optional[float] = None, timeout_sec: float = 60.0) -> bool:
        """
        阻塞执行点到点移动，直到到达目标或超时
        
        :return: 是否成功到达
        """
        self.is_active = True
        self.align_start_time = None
        self.is_aligning = False
        start_time = time.time()
        
        print(f"\n🚀 [GO_TO] 启动盲走前往目标点: X={target_x:.2f}m, Y={target_y:.2f}m, Theta={target_theta_deg}°")
        
        while self.is_active:
            now = time.time()
            if now - start_time > timeout_sec:
                print(f"⚠️ [GO_TO] 移动超时 ({timeout_sec}s)！停止移动。")
                self.stop()
                self.is_active = False
                return False

            vx, wz, is_arrived = self.compute_control(target_x, target_y, target_theta_deg)
            if is_arrived:
                print(f"🎯 [GO_TO] 已成功到达目标容差区域 (距离目标 < {self.dist_tol}m)，停车。")
                self.stop()
                self.is_active = False
                return True

            self.publish_cmd_vel(vx, wz, 0.0)
            time.sleep(0.05)  # 20Hz 控制频率

        self.stop()
        return False

    def go_to_path(self, waypoints: list, timeout_per_waypoint: float = 60.0) -> bool:
        """
        按顺序盲走遍历多航点折线路径以绕开障碍物
        
        :param waypoints: 航点列表 [(x1, y1, th1), (x2, y2, th2), ...]
        :param timeout_per_waypoint: 单段航点超时时间 (s)
        :return: 是否顺利按顺序通过所有航点到达终点
        """
        if not waypoints:
            print("⚠️ [GO_TO_PATH] 传入的航点列表为空！")
            return False

        print(f"\n🛣️ [GO_TO_PATH] 启动多航点折线路线，共 {len(waypoints)} 个关键拐角航点...")
        for i, wp in enumerate(waypoints, 1):
            tx, ty = wp[0], wp[1]
            # 只有最后一个终点才强制校准终点角 target_theta
            tth = wp[2] if i == len(waypoints) else None
            
            print(f" ├─► 正在前往中间航点 [{i}/{len(waypoints)}]: X={tx:.2f}m, Y={ty:.2f}m")
            success = self.go_to(tx, ty, target_theta_deg=tth, timeout_sec=timeout_per_waypoint)
            if not success:
                print(f"❌ [GO_TO_PATH] 航点 [{i}/{len(waypoints)}] 盲走失败！打断路线。")
                self.stop()
                return False

        print(f"🏁 [GO_TO_PATH] 所有折线航点顺利通过，已到达最终目标点！")
        return True
