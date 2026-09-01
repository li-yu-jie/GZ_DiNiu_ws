#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 🐂 机器人底盘驱动节点
=============================
订阅：
  - /cmd_vel (geometry_msgs/msg/Twist)：接收上层 Nav2/遥控下发的目标速度
发布：
  - /odom (nav_msgs/msg/Odometry)：底盘里程计话题 (含航迹积分推算)
  - /imu/data (sensor_msgs/msg/Imu)：BNO085 IMU 绝对融合四元数数据
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial
import struct
import math
import threading
import time

class DiuNiuBaseNode(Node):
    def __init__(self):
        super().__init__('diuniu_base_node')
        
        # ──────────────────────────────────────────
        # 1. 声明并读取 ROS 2 参数
        # ──────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 460800)
        self.declare_parameter('wheelbase', 1.30)  # 物理轴距 L = 1.30m
        self.declare_parameter('max_angular_speed', 2.5) # 最大角速度参考值
        self.declare_parameter('steer_rate_limit_dps', 240.0) # 转向角速率限制 (度/秒)，防单周期满舵跳变
        # 轮式里程计线速度修正：k(v) = odom_vx_scale - odom_vx_slip * |v|。
        # 下位机上报 vx 按轮径标称值换算，存在系统性比例误差。
        # 2026-08-29 以 FAST-LIO 雷达里程计为真值标定（test/odom_lidar_calib.py，
        # 0.2/0.4/0.6 m/s × 1m/2m 六组）：k≈0.936 与速度、方向均无关，
        # → scale=0.936, slip=0.0（早期卷尺"高速滑移"结论系测量噪声，已推翻）。
        # 同时修正航迹积分与 twist（EKF odom0 融合 /wheel_odom 的 vx，源头必须修）。
        self.declare_parameter('odom_vx_scale', 1.0)
        self.declare_parameter('odom_vx_slip', 0.0)
        # 原地打角目标平滑：摇杆 ADC 噪声/手颤会经线性映射 1:1 传到目标角，
        # 转向位置环每 50ms 收到抖动目标不断重新加减速，表现为"卡卡的"
        self.declare_parameter('steer_filter_alpha', 0.25)      # EMA 低通系数 (0~1，越小越平滑但跟随越慢)
        self.declare_parameter('steer_change_deadband_deg', 0.5) # 目标角变化死区 (度)，小于死区不更新
        # 默认关闭自带里程计发布：实车由 FAST-LIO/EKF 提供 /odom 与 TF，双重发布会冲突；
        # 仅无 SLAM 的纯底盘调试时才显式开启
        self.declare_parameter('pub_odom_tf', False)
        self.declare_parameter('pub_odom_topic', False)
        
        self.port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baud_rate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.steer_rate_limit_dps = self.get_parameter('steer_rate_limit_dps').value
        self.odom_vx_scale = self.get_parameter('odom_vx_scale').value
        self.odom_vx_slip = self.get_parameter('odom_vx_slip').value
        self.steer_filter_alpha = self.get_parameter('steer_filter_alpha').value
        self.steer_change_deadband = self.get_parameter('steer_change_deadband_deg').value
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        
        # ──────────────────────────────────────────
        # 2. 串口初始化与自动重连机制
        # ──────────────────────────────────────────
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.connect_serial()
            
        # ──────────────────────────────────────────
        # 3. ROS 2 话题发布与订阅
        # ──────────────────────────────────────────
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_wheel_odom = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        # /imu2/data 双发已摘除（2026-08-28）：与 /imu/data 是同一 BNO085 消息，
        # EKF 已改订 /imu/data，勿再恢复第二发布器（白白双倍序列化与 DDS 流量）
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_cmd_vel_joy = self.create_subscription(Twist, 'cmd_vel_joy', self.cmd_vel_joy_callback, 10)
        
        # ──────────────────────────────────────────
        # 3.1 TF 广播器
        # ──────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ──────────────────────────────────────────
        # 4. 里程计积分状态变量
        # ──────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # ──────────────────────────────────────────
        # 5. 启动独立串口读取线程
        # ──────────────────────────────────────────
        self.is_running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()
        
                # ──────────────────────────────────────────
        # 6. 控制指令与超时的 Watchdog 定时器
        # ──────────────────────────────────────────
        self.target_v = 0.0
        self.target_w = 0.0
        self.target_lift = 0
        self.allow_pure_rotation = False
        self.last_sent_alpha = 0.0
        self.filtered_steer_deg = 0.0  # 原地打角 EMA 滤波状态
        self.last_imu_log_sec = 0.0    # IMU 欧拉角日志上次打印时刻 (s)
        self.last_cmd_time = self.get_clock().now()
        # ★ 手柄接管仲裁：最近一次收到 /cmd_vel_joy 的时间。
        #    手柄活跃期间（0.5s 内）忽略 Nav2 的 /cmd_vel，防止两路指令 20~50Hz 交错争抢底盘。
        #    初始化为启动时刻，避免开机瞬间 Nav2 指令抢在手柄前生效。
        self.last_joy_cmd_time = self.get_clock().now()
        # 二进制模式切换看门狗（connect_serial 设置期限，收到有效帧清除）
        self._mode_switch_deadline = None
        self.cmd_send_timer = self.create_timer(0.05, self.cmd_send_timer_callback)
        
        self.get_logger().info("🚀 DiuNiu ROS 2 驱动节点已启动，正在监听底盘数据流...")

    def connect_serial(self):
        """连接串口并强制底盘进入二进制回显模式"""
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            try:
                self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
                self.get_logger().info(f"✅ 成功连接底盘串口: {self.port_name}")
                
                # 强制单片机切入 Mode 1 (二进制高效传输模式)
                self.serial_port.write(b"mode 1\r\n")
                self.serial_port.flush()
                time.sleep(0.1)
                self.serial_port.reset_input_buffer()
                # 下位机可能未响应模式切换（无应答校验）：给读循环设一个期限，
                # 超时仍无有效遥测帧则重发 mode 1（见 serial_read_loop）
                self._mode_switch_deadline = time.monotonic() + 10.0
            except Exception as e:
                self.get_logger().error(f"❌ 串口打开失败: {e}，将在后台自动尝试重连...")
                self.serial_port = None

    def send_cmd(self, cmd_data):
        """带锁安全发送指令 (兼容字符串和二进制字节)"""
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if isinstance(cmd_data, str):
                        self.serial_port.write(cmd_data.encode('utf-8'))
                    else:
                        self.serial_port.write(cmd_data)
                    self.serial_port.flush()
                    return True
                except Exception as e:
                    self.get_logger().warn(f"串口写入异常: {e}")
                    self.serial_port = None
            return False

    def send_binary_cmd(self, vx, steer_deg, lift_cmd):
        """构造并发送 14 字节的二进制控制数据包到串口"""
        # vx: float (m/s), steer_deg: float (degree), lift_cmd: uint8 (0-stop, 1-up, 2-down)
        payload = struct.pack('<ffBB', vx, steer_deg, lift_cmd, 0)
        checksum = 10
        for b in payload:
            checksum ^= b
        frame = bytearray([0x5A, 0xA5, 10]) + payload + bytearray([checksum])
        return self.send_cmd(frame)

    def cmd_vel_callback(self, msg):
        """
        处理 Nav2 自主导航下发的目标速度 (禁止原地自转)
        ★ 行为树已移除 Spin、RPP 已禁 use_rotate_to_heading，Nav2 无任何合法纯旋转输出；
        关闭 ±90° 伪自转分支，消除启动瞬间/异常帧前轮满舵扫掠风险
        ★ 手柄优先仲裁：最近 0.5s 内收到过 /cmd_vel_joy（手柄/Web 摇杆活跃）时，
        忽略 Nav2 指令，防止两路指令 20~50Hz 交错争抢底盘导致车身抖动
        """
        dt_joy = (self.get_clock().now() - self.last_joy_cmd_time).nanoseconds / 1e9
        if dt_joy < 0.5:
            self.get_logger().warn(
                "🎮 [仲裁] 手柄/Web 摇杆活跃中，忽略 Nav2 /cmd_vel 指令",
                throttle_duration_sec=2.0
            )
            return
        # 开启原地自转：允许导航在末端或大角度偏差时进行 ±90° 前轮垂直的原地旋转修正
        self.update_cmd_vel(msg, allow_pure_rotation=True)

    def cmd_vel_joy_callback(self, msg):
        """
        处理手柄下发的目标速度 (不支持原地自转，原地只打角不走车)
        ★ 记录接收时间用于手柄优先仲裁（见 cmd_vel_callback）
        """
        self.last_joy_cmd_time = self.get_clock().now()
        self.update_cmd_vel(msg, allow_pure_rotation=False)

    def update_cmd_vel(self, msg, allow_pure_rotation=False):
        # 1. 优先处理紧急停止 (通过 angular.x 通道传递)
        if msg.angular.x > 0.5:
            self.get_logger().error("🚨 [E-STOP] 收到手柄下发的紧急停止指令，底盘断电！")
            self.send_binary_cmd(0.0, 0.0, 0)
            self.send_cmd("stop\r\n")
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_lift = 0
            return

        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        
        # 2. 处理升降动作 (通过 linear.z 通道传递)
        lift_cmd = 0
        if msg.linear.z > 0.5:
            self.get_logger().info("⬆️ [LIFT] 升降上升 (up)")
            lift_cmd = 1
        elif msg.linear.z < -0.5:
            self.get_logger().info("⬇️ [LIFT] 升降下降 (down)")
            lift_cmd = 2
            
        self.target_lift = lift_cmd
        self.allow_pure_rotation = allow_pure_rotation
        self.last_cmd_time = self.get_clock().now()

    def cmd_send_timer_callback(self):
        # 计算离上一次收到指令的时间
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        
        # 保护 1：如果超过 0.2 秒没收到任何话题指令，触发看门狗自动停机并把前轮打正
        if dt > 0.2:
            self.target_v = 0.0
            self.target_w = 0.0
            self.target_lift = 0
            self.allow_pure_rotation = False
            
        # 运动学逆解（Tricycle 三轮车模型）
        v = self.target_v
        w = self.target_w
        lift_cmd = self.target_lift
        allow_pure_rotation = self.allow_pure_rotation
        
        # 1. 车辆静止状态下 (V 接近 0)，允许原地打角
        if abs(v) < 0.05:
            if allow_pure_rotation and abs(w) >= 0.05:
                # 绕后轴中心自转：前轮垂直（打角 90度），前轮线速度为 |w| * L
                alpha_deg = 90.0 if w > 0 else -90.0
                v_front = min(abs(w) * self.wheelbase, 1.2)  # 限幅：原地自转前轮速度不超过最大线速度
                self.filtered_steer_deg = alpha_deg  # 保持滤波状态连续，切回手柄打角时不跳变
            else:
                # 原地只打角不走车
                v_front = 0.0
                raw_deg = (w / self.max_angular_speed) * 90.0
                if raw_deg > 90.0: raw_deg = 90.0
                if raw_deg < -90.0: raw_deg = -90.0
                # ★ EMA 低通 + 变化死区：摇杆噪声/手颤经线性映射会产生 ±1~2° 的
                #    20Hz 目标抖动，转向环反复重新加减速导致原地打角"卡卡的"。
                #    死区与"上次实际下发值"比较，持续推杆时滤波值累积越过死区，
                #    只会合并小幅抖动，不会冻结跟随。
                self.filtered_steer_deg += self.steer_filter_alpha * (raw_deg - self.filtered_steer_deg)
                alpha_deg = self.filtered_steer_deg
                if abs(alpha_deg - self.last_sent_alpha) < self.steer_change_deadband:
                    alpha_deg = self.last_sent_alpha
        # 2. 行驶状态下，执行正常的三轮车运动学正切计算与打角限幅
        else:
            # 三轮车运动学：alpha = atan(w * L / v)，倒车时保持正确符号
            alpha_rad = math.atan((w * self.wheelbase) / v)
            alpha_deg = math.degrees(alpha_rad)

            # 允许最大打角达到 90.0°（匹配地牛机械物理转向能力上限）
            if alpha_deg > 90.0: alpha_deg = 90.0
            if alpha_deg < -90.0: alpha_deg = -90.0

            v_front = v
            # ⚠️ 行驶分支禁止套 EMA 低通！α=0.25@20Hz 会引入 ~0.5s 跟随滞后，
            #    叠加下方的 240°/s 速率限制后 Nav2/tag_align 闭环转向严重过冲画龙。
            #    EMA 只是手柄静止打角防抖（见上方静止分支），这里只同步状态防跳变。
            self.filtered_steer_deg = alpha_deg

        # 3. 转向角速率限制：每周期最大变化量 = 速率上限 × 定时器周期(0.05s)
        #    杜绝 Nav2 角速度阶跃/AMCL 位姿跳变导致的单周期满舵左右猛打（手柄原地打角同样生效）
        max_delta = self.steer_rate_limit_dps * 0.05
        if alpha_deg > self.last_sent_alpha + max_delta:
            alpha_deg = self.last_sent_alpha + max_delta
        elif alpha_deg < self.last_sent_alpha - max_delta:
            alpha_deg = self.last_sent_alpha - max_delta
        self.last_sent_alpha = alpha_deg
            
        # 只有在小车在运动、或者看门狗没有触发（手柄/导航发布中）时才限速打印日志
        if abs(v_front) > 0.01 or abs(alpha_deg) > 1.0 or dt <= 0.2:
            self.get_logger().info(
                f"🔍 [底盘发包] v_front={v_front:.3f} m/s, alpha={alpha_deg:.2f}°, lift={lift_cmd} | dt={dt:.2f}s",
                throttle_duration_sec=0.5
            )

        # 发送 14 字节的二进制控制数据包
        self.send_binary_cmd(v_front, alpha_deg, lift_cmd)

    def serial_read_loop(self):
        """
        高频循环读取二进制流（支持自动重连和包头对齐防卡死）
        52字节结构： Header(2字节) + Len(1) + Vx(4) + Wz(4) + IMU1(16) + IMU2(16) + M1(4) + M2(4) + Checksum(1)
        """
        # B = uint8, f = float (4字节), i = int32 (4字节)
        packet_format = '<B 2f 4f 4f 2i B' 
        buffer = bytearray()
        
        while self.is_running and rclpy.ok():
            ser_obj = None
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    ser_obj = self.serial_port
                    
            if ser_obj is None:
                # 串口断开，自动进行重连尝试
                time.sleep(1.0)
                self.connect_serial()
                continue

            # 模式切换看门狗：连接后 10s 仍无有效遥测帧，
            # 说明下位机可能没切到二进制模式（无应答校验），重发 mode 1
            if (self._mode_switch_deadline is not None
                    and time.monotonic() > self._mode_switch_deadline):
                self.get_logger().warn("连接后无有效遥测帧，重发 mode 1 切换二进制模式")
                self.send_cmd("mode 1\r\n")
                self._mode_switch_deadline = time.monotonic() + 5.0
                
            try:
                waiting = ser_obj.in_waiting
                if waiting > 0:
                    data = ser_obj.read(waiting)
                    buffer.extend(data)
                else:
                    time.sleep(0.005)
                    continue

                # 循环解析缓冲区
                while len(buffer) >= 2:
                    if buffer[0] == 0x5A and buffer[1] == 0xA5:
                        if len(buffer) < 52:
                            break  # 帧不够完整，等待下一次读取
                        
                        # 提取包体（除去包头的后 50 字节）
                        payload_data = buffer[2:52]
                        
                        # 进行异或校验和计算
                        calc_crc = 0
                        for byte in payload_data[:-1]:
                            calc_crc ^= byte
                        
                        recv_crc = payload_data[-1]
                        
                        if calc_crc == recv_crc:
                            # 校验通过，解包
                            parsed = struct.unpack(packet_format, payload_data)

                            length = parsed[0]
                            if length != 48:
                                # Length 字段不符（协议固定 48）：CRC 碰巧通过的其他
                                # 帧/控制回显，不能当遥测解析，否则数据错乱
                                self.get_logger().warn(
                                    f"Length 字段异常: {length}（期望 48），丢弃该帧",
                                    throttle_duration_sec=5.0)
                                del buffer[:52]
                                continue
                            vx = parsed[1]
                            wz = parsed[2]
                            # 帧内 IMU1 槽位无真实数据(恒定单位四元数)，BNO085 实际挂在 IMU2 槽位
                            imu2_qw, imu2_qx, imu2_qy, imu2_qz = parsed[7:11]

                            self.publish_sensor_data(vx, wz, imu2_qw, imu2_qx, imu2_qy, imu2_qz)
                            self._mode_switch_deadline = None   # 收到有效帧，模式切换确认成功
                            del buffer[:52]
                        else:
                            # 校验和错误，可能是文本段撞字符，丢弃头部并继续寻找
                            del buffer[0:1]
                    else:
                        # 非二进制包头字符，丢弃
                        del buffer[0:1]
            except Exception as e:
                self.get_logger().error(f"串口读取/解析异常: {e}")
                with self.serial_lock:
                    # 只置 None 不 close 会泄漏底层 fd，重连时端口可能仍被占用
                    try:
                        if self.serial_port is not None:
                            self.serial_port.close()
                    except Exception:
                        pass
                    self.serial_port = None
                time.sleep(0.5)

    def publish_sensor_data(self, vx, wz, qw, qx, qy, qz):
        """
        里程计航迹推算并发布 Odom 和 Imu 话题
        """
        # 速度相关修正 k(v) = scale - slip*|v|（见参数声明注释的标定数据）
        k = self.odom_vx_scale - self.odom_vx_slip * abs(vx)
        vx *= max(0.0, k)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. dt 异常拦截 (防开机首帧大 dt 冲激)
        if dt <= 0.0 or dt > 1.0:
            if dt > 1.0:
                self.get_logger().warn(
                    f"⚠️ [里程计] 检测到异常 dt={dt:.3f}s（串口卡顿/调度延迟），本帧位移被丢弃",
                    throttle_duration_sec=2.0)
            dt = 0.0

        # 2. 中点弧线积分 (Midpoint Runge-Kutta 2nd Order Integration)
        # 相比传统欧拉积分，中点积分能精准还原转弯时的弧线运动，消除过弯积算误差
        delta_th = wz * dt
        half_th = self.th + delta_th / 2.0
        self.x += vx * math.cos(half_th) * dt
        self.y += vx * math.sin(half_th) * dt
        self.th += delta_th

        # 调试日志：每秒打印一次从串口收到的实际速度与积分出的坐标
        self.get_logger().info(
            f"📊 [里程计反馈] 收到实际速度: vx={vx:.3f} m/s, wz={wz:.3f} rad/s | 积分坐标: x={self.x:.3f}, y={self.y:.3f}, th={self.th:.3f}",
            throttle_duration_sec=1.0
        )

        # 发布 ODOM 话题
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 设置里程计物理位置 (X, Y)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # 将累计角度偏航角 th 转为四元数
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)

        # 设置速度 (Vx, Wz)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        # 设置位姿和速度协方差矩阵 (标准轮式里程计协方差)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.99, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.99, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.99, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.99, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.99, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.99, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        # 始终发布独立的轮式里程计话题 /wheel_odom (供监控、融合及可视化使用)
        self.pub_wheel_odom.publish(odom)

        # 当参数允许时，同步发布 /odom 话题
        if self.pub_odom_topic:
            self.pub_odom.publish(odom)

        # 发布 TF 变换 (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        if self.get_parameter('pub_odom_tf').value:
            self.tf_broadcaster.sendTransform(t)

        # 2. 发布 IMU 话题 (使用 BNO085 的高频融合绝对姿态四元数)
        imu = Imu()
        imu.header.stamp = current_time.to_msg()
        # BNO085 在底盘主控板上，与雷达内置 IMU (imu_link) 是两个器件，frame 勿混用
        imu.header.frame_id = 'base_imu_link'

        # 严格遵守 ROS 2 官方坐标系标准赋值 (x, y, z, w)
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        # EKF 融合要求非零协方差；BNO085 融合姿态典型精度取 roll/pitch 0.05、yaw 0.02 (rad²)
        imu.orientation_covariance = [
            0.05, 0.0,  0.0,
            0.0,  0.05, 0.0,
            0.0,  0.0,  0.02
        ]
        # 单片机帧内无陀螺仪/加速度计原始数据，按 ROS 约定首元素置 -1 表示“不提供该量”
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)

        # 3. 每 2 秒限频打印一次 IMU 的四元数与解算出的欧拉角 (Roll, Pitch, Yaw)
        #    rclpy 节流只抑制输出不抑制参数构造，三角解算包在时间判断里，非打印帧零开销
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_imu_log_sec >= 2.0:
            self.last_imu_log_sec = now_sec
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.degrees(math.asin(sinp)) if abs(sinp) < 1.0 else math.degrees(math.copysign(math.pi / 2.0, sinp))

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

            self.get_logger().info(
                f"🧭 [IMU 反馈] 四元数: [w={qw:.4f}, x={qx:.4f}, y={qy:.4f}, z={qz:.4f}] | 欧拉角: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°"
            )

    def destroy_node(self):
        self.is_running = False
        self.read_thread.join()
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(b"mode 0\r\n") # 退出时友好地让单片机切回文本模式，方便串口助手调试
                    self.serial_port.flush()
                    self.serial_port.close()
                except Exception:
                    pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DiuNiuBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
