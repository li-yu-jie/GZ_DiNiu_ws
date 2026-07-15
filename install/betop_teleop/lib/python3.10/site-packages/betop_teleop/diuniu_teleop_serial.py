#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘手柄遥控节点 - 串口直控模式
=======================================
功能：
  - 订阅 ROS 2 /joy 话题，读取手柄数据
  - 独占串口，直接通过 14 字节二进制控制包协议高频向 STM32 下发控制量
  - 后台线程接收 52 字节二进制包与文本日志
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import threading
import struct

class DiuNiuTeleopSerial(Node):
    def __init__(self):
        super().__init__('diuniu_teleop_serial')

        # ──────────────────────────────────────────
        # 声明 ROS 2 参数
        # ──────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 460800)
        self.declare_parameter('max_linear_speed', 1.2)
        self.declare_parameter('max_steer_angle', 95.0)   # 物理前轮打角极限度数
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_steer', 2)
        self.declare_parameter('enable_button', 7)
        self.declare_parameter('stop_button', 1)
        self.declare_parameter('button_lift_up', 4)
        self.declare_parameter('button_lift_down', 0)
        self.declare_parameter('steer_invert', False)
        self.declare_parameter('publish_rate', 20.0)

        # ──────────────────────────────────────────
        # 读取参数
        # ──────────────────────────────────────────
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.max_speed = self.get_parameter('max_linear_speed').value
        self.max_steer_deg = self.get_parameter('max_steer_angle').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_steer = self.get_parameter('axis_steer').value
        self.enable_button = self.get_parameter('enable_button').value
        self.stop_button = self.get_parameter('stop_button').value
        self.button_lift_up = self.get_parameter('button_lift_up').value
        self.button_lift_down = self.get_parameter('button_lift_down').value
        self.steer_invert = self.get_parameter('steer_invert').value
        self.rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"🚀 [串口直控手柄节点] 正在初始化 串口: {self.port} @ {self.baud}")
        self.get_logger().info(f"配置参数: 最大线速度={self.max_speed} m/s, 最大前轮偏角={self.max_steer_deg}°")

        # ──────────────────────────────────────────
        # 状态变量
        # ──────────────────────────────────────────
        self.latest_joy_msg = None
        self.last_joy_time = 0.0
        self.ser = None
        self.serial_lock = threading.Lock()

        # ──────────────────────────────────────────
        # 连接串口与初始化工作线程
        # ──────────────────────────────────────────
        self.connect_serial()
        self.keep_reading = True
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        # 订阅 /joy 话题
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 周期控制下发定时器
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.control_timer_callback)

    def connect_serial(self):
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f"✅ 串口 {self.port} 连接成功！")
                
                # 连上后自动切换单片机为二进制遥测回显模式
                self.ser.write(b"mode 1\r\n")
                self.ser.flush()
            except Exception as e:
                self.get_logger().error(f"❌ 串口 {self.port} 打开失败: {e}，将在发送时自动重连。")
                self.ser = None

    def joy_callback(self, msg):
        self.latest_joy_msg = msg
        self.last_joy_time = time.time()

    def send_binary_command(self, vx, steer_deg, lift_cmd):
        """构造并发送 14 字节的二进制控制数据包"""
        with self.serial_lock:
            if self.ser is None or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                except Exception:
                    return False
            try:
                # 封装 10 字节 Payload (vx float, steer_deg float, lift_cmd uint8, reserve uint8=0)
                payload = struct.pack('<ffBB', vx, steer_deg, lift_cmd, 0)
                
                # 计算校验和 (从 Length=10 开始一直异或到 Reserve)
                checksum = 10
                for b in payload:
                    checksum ^= b
                    
                # 拼装完整包 (14字节)
                frame = bytearray([0x5A, 0xA5, 10]) + payload + bytearray([checksum])
                self.ser.write(frame)
                self.ser.flush()
                return True
            except Exception as e:
                self.get_logger().warn(f"串口写入异常: {e}，尝试重连。")
                self.ser = None
                return False

    def send_text_cmd(self, cmd_str):
        """串口发送文本指令"""
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd_str.encode('utf-8'))
                    self.ser.flush()
                except Exception:
                    self.ser = None

    def control_timer_callback(self):
        now = time.time()

        # ── 保护 1：信号超时 ──────────────────────────
        if self.latest_joy_msg is None or (now - self.last_joy_time > 0.5):
            self.get_logger().warn("手柄信号丢失或未连接，发送紧急停止指令！", throttle_duration_sec=3.0)
            self.send_binary_command(0.0, 0.0, 0)
            self.send_text_cmd("stop\r\n")
            return

        msg = self.latest_joy_msg

        # ── 保护 2：急停按键（B 键）────────────────────
        is_stop_pressed = (len(msg.buttons) > self.stop_button
                           and msg.buttons[self.stop_button] == 1)
        if is_stop_pressed:
            self.get_logger().error("手柄触发紧急停止 (B键)！")
            self.send_binary_command(0.0, 0.0, 0)
            self.send_text_cmd("stop\r\n")
            return

        # ── 保护 3：使能键（LB/RB 键）──────────────────
        is_enabled = (len(msg.buttons) > self.enable_button
                      and msg.buttons[self.enable_button] == 1)

        if is_enabled:
            # 读取摇杆
            joy_linear = msg.axes[self.axis_linear] if len(msg.axes) > self.axis_linear else 0.0
            joy_steer  = msg.axes[self.axis_steer]  if len(msg.axes) > self.axis_steer  else 0.0

            # 映射行驶参数
            target_speed = joy_linear * self.max_speed
            steer_sign = -1.0 if self.steer_invert else 1.0
            target_deg = joy_steer * self.max_steer_deg * steer_sign

            if abs(target_speed) < 0.05:
                target_speed = 0.0
            if abs(target_deg) < 8.0:
                target_deg = 0.0

            # 升降点动命令 (1-上，2-下，0-停)
            lift_cmd = 0
            if len(msg.buttons) > self.button_lift_up and msg.buttons[self.button_lift_up] == 1:
                lift_cmd = 1
            elif len(msg.buttons) > self.button_lift_down and msg.buttons[self.button_lift_down] == 1:
                lift_cmd = 2

            # 发送二进制控制帧
            self.send_binary_command(target_speed, target_deg, lift_cmd)

            # 限速打印调试信息
            self.get_logger().info(
                f"[串口直控二进制] 目标线速={target_speed:+.3f} m/s | 转向角度={target_deg:+.1f}° | 升降命令={lift_cmd}",
                throttle_duration_sec=0.5
            )
        else:
            # 松开 LB 键，发送零速度与停止
            self.send_binary_command(0.0, 0.0, 0)

    def read_serial_loop(self):
        """持续从串口读取数据，兼容 52 字节二进制遥测帧与 ASCII 文本"""
        buffer = bytearray()
        last_log_time = 0.0

        while rclpy.ok() and self.keep_reading:
            ser_obj = None
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    ser_obj = self.ser

            if ser_obj:
                try:
                    waiting = ser_obj.in_waiting
                    if waiting > 0:
                        data = ser_obj.read(waiting)
                        buffer.extend(data)

                    while len(buffer) >= 2:
                        # 1. 解析二进制遥测帧 (52 字节)
                        if buffer[0] == 0x5A and buffer[1] == 0xA5:
                            if len(buffer) < 52:
                                break  # 字节数不够，等待下一次

                            # 校验长度字段 (Byte 2 必须是 48)
                            if buffer[2] != 48:
                                del buffer[:2]
                                continue

                            # 校验和
                            calc_sum = 0
                            for b in buffer[2:51]:
                                calc_sum ^= b

                            if calc_sum == buffer[51]:
                                payload = buffer[3:51]
                                # 解析：vx, wz, q1[4], q2[4], m1_pos, m2_pos
                                vx, wz = struct.unpack('<ff', payload[:8])
                                m1_pos, m2_pos = struct.unpack('<ii', payload[40:48])

                                now = time.time()
                                if now - last_log_time >= 0.2:
                                    self.get_logger().info(
                                        f"[直控-底盘遥测] Vx={vx:+.3f} m/s, Wz={wz:+.3f} rad/s | M1_Pos={m1_pos}, M2_Pos={m2_pos}"
                                    )
                                    last_log_time = now

                                del buffer[:52]
                            else:
                                del buffer[0:1]  # 校验和错误，剔除包头重新对齐
                        else:
                            # 2. 解析普通文本行
                            if 0x0A in buffer:
                                idx = buffer.index(0x0A)
                                line_bytes = buffer[:idx+1]
                                try:
                                    line = line_bytes.decode('utf-8', errors='ignore').strip()
                                    if line:
                                        self.get_logger().info(f"[底盘串口回显]: {line}")
                                except Exception:
                                    pass
                                del buffer[:idx+1]
                            else:
                                if len(buffer) > 1000:
                                    del buffer[:500]
                                break
                except Exception:
                    pass

            time.sleep(0.01)

    def destroy_node(self):
        self.keep_reading = False
        self.get_logger().info("正在关闭串口直控手柄节点...")
        self.send_binary_command(0.0, 0.0, 0)
        self.send_text_cmd("stop\r\n")
        time.sleep(0.1)
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DiuNiuTeleopSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
