#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼底盘极简移动控制接口 (Path Driver Control Interface)
===========================================================
供用户直接调用的直观接口函数：
  move_to(target_x, target_y, target_theta_deg, speed=0.5, mode="forward")
  move_path(path_list)
支持：
  - 前进 (mode="forward") / 倒车 (mode="reverse")
  - 目标坐标 X, Y (米) 与目标角度 Theta (度)
  - 自定义线速度 speed (m/s)
"""

import math
import time
from typing import List, Tuple, Optional, Callable, Union

try:
    from Path.location_table import normalize_angle_deg, normalize_angle_rad, calc_distance_and_heading, get_location, WAYPOINT_TABLE
    from Path.ackermann_odometry import AckermannOdometry
except ModuleNotFoundError:
    from location_table import normalize_angle_deg, normalize_angle_rad, calc_distance_and_heading, get_location, WAYPOINT_TABLE
    from ackermann_odometry import AckermannOdometry


class DirectMoveController:
    def __init__(self, 
                 odometry: AckermannOdometry,
                 cmd_vel_pub_func: Optional[Callable[[float, float, float], None]] = None,
                 wheelbase: float = 1.30,
                 kp_angular: float = 1.2,
                 dist_tolerance: float = 0.20,
                 angle_tolerance_deg: float = 5.0):
        """
        :param odometry: 里程计死算对象
        :param cmd_vel_pub_func: 发送 /cmd_vel 回调函数 func(vx, wz, vz_lift)
        """
        self.odom = odometry
        self.cmd_vel_pub = cmd_vel_pub_func
        self.wheelbase = wheelbase
        self.kp_w = kp_angular
        self.dist_tol = dist_tolerance
        self.angle_tol_deg = angle_tolerance_deg
        
        self.is_active = False
        self.is_aligning = False

    def publish_cmd_vel(self, vx: float, wz: float, vz_lift: float = 0.0):
        if self.cmd_vel_pub:
            self.cmd_vel_pub(vx, wz, vz_lift)

    def stop(self):
        self.publish_cmd_vel(0.0, 0.0, 0.0)

    def move_to(self, 
                target_x: float, 
                target_y: float, 
                target_theta_deg: float, 
                speed: float = 0.5, 
                mode: str = "forward", 
                timeout_sec: float = 60.0) -> bool:
        """
        单点移动基础调用函数
        
        :param target_x: 目标 X 坐标 (m)
        :param target_y: 目标 Y 坐标 (m)
        :param target_theta_deg: 目标到达角度 (度)
        :param speed: 设定移动速度 (m/s, 取正数)
        :param mode: "forward" (前进) 或 "reverse" (倒车)
        :param timeout_sec: 超时时间 (s)
        :return: 是否成功到达
        """
        self.is_active = True
        speed = abs(speed)
        mode = mode.lower().strip()
        start_time = time.time()
        
        dir_str = "前进 ⬆️" if mode == "forward" else "倒车 ⬇️"
        print(f"\n🚙 [MOVE_TO] 开启【{dir_str}】移动 -> 目标: X={target_x:.2f}m, Y={target_y:.2f}m, Theta={target_theta_deg}° (速度={speed}m/s)")
        
        align_start_time = None
        self.is_aligning = False

        last_print_time = 0.0
        while self.is_active:
            now = time.time()
            if now - start_time > timeout_sec:
                print(f"\n⚠️ [MOVE_TO] 移动超时 ({timeout_sec}s)！未能在规定时间内抵达目标。")
                self.stop()
                self.is_active = False
                return False

            curr_x, curr_y, curr_th_deg = self.odom.get_pose()
            curr_th_rad = math.radians(curr_th_deg)
            
            # 1. 距离与相对向量计算
            dist, target_heading_deg, target_heading_rad = calc_distance_and_heading(curr_x, curr_y, target_x, target_y)
            
            # 2. 到达距离判定 (到点后，原地旋转对正终点角度，然后进入下一个点)
            if dist <= self.dist_tol:
                # 记录首次进入角度校准的时间
                if align_start_time is None:
                    align_start_time = now

                angle_err_deg = normalize_angle_deg(target_theta_deg - curr_th_deg)

                # 安全保护：微调时间强制限制（最多 8.0s）
                if now - align_start_time > 8.0:
                    print(f"\n⚠️ [终点对齐] 原地自转微调已超时 (8.0s)，为防撞击中止调整，直接判定到达！")
                    self.stop()
                    self.is_active = False
                    return True

                if abs(angle_err_deg) <= self.angle_tol_deg:
                    print(f"\n🎯 [MOVE_TO] 已精准抵达目标点并完成最终角度校准！(X={curr_x:.2f}, Y={curr_y:.2f}, Theta={curr_th_deg:.1f}°)")
                    self.stop()
                    self.is_active = False
                    return True
                else:
                    # 原地对齐终点姿态角 (vx=0.0，限制最大旋转速度为 0.45 rad/s，以克服重载摩擦力)
                    wz_val = self.kp_w * math.radians(angle_err_deg)
                    wz = max(-0.45, min(0.45, wz_val))
                    if abs(wz) < 0.15:
                        wz = math.copysign(0.15, angle_err_deg)
                        
                    if now - last_print_time >= 1.0:
                        last_print_time = now
                        print(f"🔄 [终点对齐] 当前角度: {curr_th_deg:.1f}°, 目标终点角: {target_theta_deg:.1f}°, 偏差: {angle_err_deg:.1f}° | 发送 wz={wz:.2f}")
                    self.publish_cmd_vel(0.0, wz, 0.0)
                    time.sleep(0.05)
                    continue

            # 3. 前进 vs 倒车行驶运动学计算与航向对正判定
            if mode == "forward":
                target_yaw_rad = target_heading_rad
            else:  # reverse (倒车)
                # 倒车时，车尾向目标点靠近，期望航向角偏移 180°
                target_yaw_rad = normalize_angle_rad(target_heading_rad - math.pi)

            heading_err_rad = normalize_angle_rad(target_yaw_rad - curr_th_rad)
            heading_err_deg = math.degrees(heading_err_rad)

            # 引入迟滞环机制进行原地对正与行驶切换，防状态在边界线上频繁交错抖动
            if self.is_aligning:
                if abs(heading_err_deg) < 5.0:
                    self.is_aligning = False
                    print(f"\n🎉 [原地对正] 航向已对准到 {heading_err_deg:.1f}°，退出自转并切入行驶！")
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
                
                if now - last_print_time >= 1.0:
                    last_print_time = now
                    print(f"🔄 [原地对正] 当前角度: {curr_th_deg:.1f}°, 目标朝向: {math.degrees(target_yaw_rad):.1f}°, 偏差: {heading_err_deg:.1f}° | 发送 wz={wz:.2f}")
            else:
                # 航向已对准，直行 (引入 1.5度控制死区，防止 IMU/编码器噪声导致前轮高频抖动)
                # 接近目标线性减速，避免恒速冲进容差圈后过冲 → 反复"停-转-走"振荡
                v_mag = min(speed, max(0.10, 1.2 * dist))
                vx = v_mag if mode == "forward" else -v_mag
                if abs(heading_err_deg) <= 1.5:
                    wz = 0.0
                else:
                    wz = self.kp_w * heading_err_rad
                    wz = max(-0.20, min(0.20, wz))

                # 定时打印诊断信息
                if now - last_print_time >= 1.0:
                    last_print_time = now
                    summary = self.odom.get_summary()
                    print(f"🚙 [行驶中] 当前坐标: ({curr_x:.2f}, {curr_y:.2f}, {curr_th_deg:.1f}°) "
                          f"| 距目标: {dist:.2f}m | 输出控制: vx={vx:.2f}m/s, wz={wz:.2f}rad/s "
                          f"| 里程计诊断: 已更新={summary['update_count']}次, 已连IMU={summary['has_imu']}")

            self.publish_cmd_vel(vx, wz, 0.0)
            time.sleep(0.05)

        self.stop()
        return False

    def move_path(self, path_list: List[Tuple[float, float, float, float, str]], timeout_per_step: float = 60.0) -> bool:
        """
        顺序执行您自己规划的路线航点列表
        
        path_list 格式: [
            (x1, y1, theta1, speed1, "forward"),
            (x2, y2, theta2, speed2, "reverse"),
            ...
        ]
        """
        print(f"\n🛣️ [MOVE_PATH] 正在顺序执行您规划的 {len(path_list)} 段路线...")
        for i, step in enumerate(path_list, 1):
            tx, ty, tth = step[0], step[1], step[2]
            spd = step[3] if len(step) > 3 else 0.5
            md = step[4] if len(step) > 4 else "forward"
            
            print(f"\n▶️ [路线段 {i}/{len(path_list)}]: 前往 (X={tx}, Y={ty}, Theta={tth}°), 模式={md}, 速度={spd}m/s")
            success = self.move_to(tx, ty, tth, speed=spd, mode=md, timeout_sec=timeout_per_step)
            if not success:
                print(f"❌ [MOVE_PATH] 路线段 {i} 执行失败！已停机。")
                self.stop()
                return False

        print(f"🎉 [MOVE_PATH] 用户规划的所有路线段已全部顺利完成！")
        return True

    def move_to_point(self, 
                      point_name: Union[str, int], 
                      speed: float = 0.5, 
                      mode: str = "forward", 
                      timeout_sec: float = 60.0) -> bool:
        """
        按坐标点名直接移动 (自动查表读取坐标数据 X, Y, Theta)
        
        :param point_name: 坐标点名，例如: 1, 2, 3, "parking", "aisle_entry" 等
        :param speed: 移动线速度 (m/s)
        :param mode: "forward" (前进) 或 "reverse" (倒车)
        :param timeout_sec: 超时时间 (s)
        :return: 是否成功到达
        """
        try:
            from Path.location_table import get_location, WAYPOINT_TABLE
        except ModuleNotFoundError:
            from location_table import get_location, WAYPOINT_TABLE
        
        # 1. 尝试从作业点位表获取坐标数据
        loc = get_location(point_name)
        if loc is None and str(point_name) in WAYPOINT_TABLE:
            loc = WAYPOINT_TABLE[str(point_name)]

        if loc is None:
            print(f"❌ [MOVE_TO_POINT] 未找到点名 '{point_name}' 的坐标数据！")
            return False

        tx, ty, tth = loc[0], loc[1], loc[2]
        print(f"📍 [点名查表] 点名 '{point_name}' -> 坐标数据: X={tx:.2f}m, Y={ty:.2f}m, Theta={tth}°")
        return self.move_to(tx, ty, tth, speed=speed, mode=mode, timeout_sec=timeout_sec)

    def move_path_by_names(self, 
                           name_list: list, 
                           speed: float = 0.5, 
                           mode: str = "forward", 
                           timeout_per_step: float = 60.0) -> bool:
        """
        按输入的坐标点名列表顺序移动
        
        name_list 格式支持:
          - 纯点名列表: ["parking", 1, 3]
          - 带速度/方向元组: [("parking", 0.5, "forward"), (1, 0.4, "reverse")]
        """
        print(f"\n🛣️ [MOVE_BY_NAMES] 正在按输入的 {len(name_list)} 个点名顺序移动...")
        for i, item in enumerate(name_list, 1):
            if isinstance(item, (tuple, list)):
                p_name = item[0]
                spd = item[1] if len(item) > 1 else speed
                md = item[2] if len(item) > 2 else mode
            else:
                p_name = item
                spd = speed
                md = mode

            print(f"\n▶️ [步骤 {i}/{len(name_list)}]: 点名 '{p_name}'")
            success = self.move_to_point(p_name, speed=spd, mode=md, timeout_sec=timeout_per_step)
            if not success:
                print(f"❌ [MOVE_BY_NAMES] 步骤 {i} ({p_name}) 执行失败！")
                self.stop()
                return False

        print(f"🎉 [MOVE_BY_NAMES] 所有输入的点名路径已成功跑完！")
        return True
