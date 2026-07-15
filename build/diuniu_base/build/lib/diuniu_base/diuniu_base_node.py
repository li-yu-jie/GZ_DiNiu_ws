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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 460800)
        self.declare_parameter('wheelbase', 1.30)  # 物理轴距 L = 1.30m
        self.declare_parameter('max_angular_speed', 2.5) # 最大角速度参考值
        
        self.port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baud_rate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
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
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
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
        处理手柄/Nav2下发的目标速度与控制动作
        """
        # 1. 优先处理紧急停止 (通过 angular.x 通道传递)
        if msg.angular.x > 0.5:
            self.get_logger().error("🚨 [E-STOP] 收到手柄下发的紧急停止指令，底盘断电！")
            self.send_binary_cmd(0.0, 0.0, 0)
            self.send_cmd("stop\r\n")
            return

        # 2. 处理升降动作 (通过 linear.z 通道传递)
        lift_cmd = 0
        if msg.linear.z > 0.5:
            self.get_logger().info("⬆️ [LIFT] 升降上升 (up)")
            lift_cmd = 1
        elif msg.linear.z < -0.5:
            self.get_logger().info("⬇️ [LIFT] 升降下降 (down)")
            lift_cmd = 2

        # 3. 正常行驶运动学逆解（Tricycle 三轮车模型），并发给 STM32
        v = msg.linear.x
        w = msg.angular.z
        
        # 1. 车辆静止状态下 (V 接近 0)，允许原地打角（以调整前轮偏角），但驱动轮速度保持为 0
        if abs(v) < 0.05:
            v_front = 0.0
            # 使用配置的最大角速度进行归一化映射，确保推满摇杆时能够打满物理极限 95 度
            alpha_deg = (w / self.max_angular_speed) * 95.0
            if alpha_deg > 95.0: alpha_deg = 95.0
            if alpha_deg < -95.0: alpha_deg = -95.0
        # 2. 行驶状态下，执行正常的三轮车运动学正切计算与打角限幅
        else:
            alpha_rad = math.atan((w * self.wheelbase) / v)
            alpha_deg = math.degrees(alpha_rad)
            # 避免除以 cos(alpha) 导致的转向时速度奇异激增，直接令前轮驱动速度等于目标线速度
            v_front = v
            
            # 软件物理限角保护限制在 [-95°, +95°] 内
            if alpha_deg > 95.0: alpha_deg = 95.0
            if alpha_deg < -95.0: alpha_deg = -95.0
            
        # 调试日志：实时打印输入输出以确诊计算问题
        self.get_logger().info(
            f"🔍 [解算调试] v={v:.3f}, w={w:.3f} -> 前轮速={v_front:.3f} m/s, 前轮角={alpha_deg:.2f}°",
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
                            vx = parsed[1]
                            wz = parsed[2]
                            imu1_qw, imu1_qx, imu1_qy, imu1_qz = parsed[3:7]
                            
                            self.publish_sensor_data(vx, wz, imu1_qw, imu1_qx, imu1_qy, imu1_qz)
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
                    self.serial_port = None
                time.sleep(0.5)

    def publish_sensor_data(self, vx, wz, qw, qx, qy, qz):
        """
        里程计航迹推算并发布 Odom 和 Imu 话题
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. 航迹推算 (Odometry Integration)
        self.th += wz * dt
        self.x += vx * math.cos(self.th) * dt
        self.y += vx * math.sin(self.th) * dt

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
        self.pub_odom.publish(odom)

        # 2. 发布 IMU 话题 (使用 BNO085 的高频融合绝对姿态四元数)
        imu = Imu()
        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = 'imu_link'
        
        # 严格遵守 ROS 2 官方坐标系标准赋值 (x, y, z, w)
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        self.pub_imu.publish(imu)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
