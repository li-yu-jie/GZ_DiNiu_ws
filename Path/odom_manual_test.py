#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼底盘里程计手动推车闭环测试节点 / 脚本
=============================================
功能：
  1. 订阅 ROS 2 话题 (/imu/data, /wheel_odom) 或直接通过串口解析底层数据；
  2. 实时进行基于 IMU 航向角 + 编码器步进的中点弧线死算；
  3. 在终端高频/定频格式化打印车辆当前的绝对坐标 (X, Y, Theta) 与运动统计；
  4. 支持交互控制指令：
     - 按 'r' 键: 重置坐标至零点 (0.0, 0.0, 0.0)
     - 按 '1'~'6' 键: 校准/强设坐标至对应的 1~6 号作业点位
     - 按 'c' 键: 清空累计里程
     - 按 'q' 键: 退出测试

使用方法:
  1. ROS 2 环境下运行:
     python3 Path/odom_manual_test.py --mode ros2
  2. 串口直连模式:
     python3 Path/odom_manual_test.py --mode serial --port /dev/ttyUSB0 --baud 460800
  3. 模拟测试模式:
     python3 Path/odom_manual_test.py --mode sim
"""

import sys
import os
import time
import math
import struct
import select
import termios
import tty
import argparse
import threading
from typing import Optional

# 将当前工作目录添加到 sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from Path.location_table import LOCATION_TABLE, get_location, calc_distance_and_heading, normalize_angle_deg
    from Path.ackermann_odometry import AckermannOdometry
except ModuleNotFoundError:
    from location_table import LOCATION_TABLE, get_location, calc_distance_and_heading, normalize_angle_deg
    from ackermann_odometry import AckermannOdometry


class NonBlockingKeyboard:
    """非阻塞终端键盘输入监听器"""
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_char(self) -> Optional[str]:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class OdomManualTestRunner:
    def __init__(self, mode: str = "ros2", serial_port: str = "/dev/ttyUSB0", baud_rate: int = 460800):
        self.mode = mode
        self.serial_port_name = serial_port
        self.baud_rate = baud_rate
        self.is_running = True
        
        # 实例化死算里程计核心类
        self.odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        
        # 实时传感器观测缓存
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.latest_imu_yaw_deg = 0.0
        self.last_update_time = time.time()
        
        self.ros_node = None

    def start(self):
        print("\n" + "=" * 65)
        print("  🐂 🐂 地牛自动叉车 - 阿克曼底盘里程计手动推车闭环测试 🐂 🐂")
        print("=" * 65)
        print(f"运行模式: {self.mode.upper()}")
        print("快捷键说明: [r] 零点重置 | [1-6] 设为1-6号点位 | [c] 里程清零 | [q] 退出测试\n")
        
        if self.mode == "ros2":
            self._start_ros2()
        elif self.mode == "serial":
            self._start_serial()
        elif self.mode == "sim":
            self._start_sim()
        else:
            print(f"❌ 未知的模式: {self.mode}")
            return

        # 启动控制终端 UI 循环
        self._interactive_loop()

    def _start_ros2(self):
        try:
            import rclpy
            from rclpy.node import Node
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
        except ImportError:
            print("⚠️ 未检测到 ROS 2 环境 (rclpy)，自动降级至串口/模拟模式")
            self._start_sim()
            return

        rclpy.init()
        
        class OdomTestROSNode(Node):
            def __init__(outer_self):
                super().__init__('odom_manual_test_node')
                outer_self.sub_imu = outer_self.ros_node_create_sub(
                    self, Imu, '/imu/data', outer_self.imu_cb
                )
                outer_self.sub_wheel = outer_self.ros_node_create_sub(
                    self, Odometry, '/wheel_odom', outer_self.wheel_cb
                )

            def imu_cb(outer_self, msg):
                # 提取四元数 Yaw
                qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
                self.latest_imu_yaw_deg = yaw_deg
                self.odom.set_imu_yaw(yaw_deg)

            def wheel_cb(outer_self, msg):
                now = time.time()
                dt = now - self.last_update_time
                self.last_update_time = now
                
                vx = msg.twist.twist.linear.x
                self.current_vx = vx
                self.current_wz = msg.twist.twist.angular.z
                
                self.odom.update(vx=vx, dt=dt, raw_imu_yaw_deg=self.latest_imu_yaw_deg)

            def ros_node_create_sub(outer_self, node_inst, msg_type, topic, cb):
                return node_inst.create_subscription(msg_type, topic, cb, 10)

        self.ros_node = OdomTestROSNode()
        
        # ROS 2 Spin 后台线程
        def ros_spin():
            rclpy.spin(self.ros_node)

        t = threading.Thread(target=ros_spin, daemon=True)
        t.start()
        print("✅ ROS 2 节点已启动，订阅 /imu/data 与 /wheel_odom ...")

    def _start_serial(self):
        import serial
        
        def serial_loop():
            try:
                ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.1)
                ser.write(b"mode 1\r\n")
                ser.flush()
                print(f"✅ 成功连接串口 {self.serial_port_name} ({self.baud_rate})")
            except Exception as e:
                print(f"❌ 打开串口失败: {e}")
                return

            packet_format = '<B 2f 4f 4f 2i B'
            buffer = bytearray()
            
            while self.is_running:
                try:
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting)
                        buffer.extend(data)
                    else:
                        time.sleep(0.005)
                        continue

                    while len(buffer) >= 52:
                        if buffer[0] == 0x5A and buffer[1] == 0xA5:
                            payload = buffer[2:52]
                            calc_crc = 0
                            for b in payload[:-1]:
                                calc_crc ^= b
                            if calc_crc == payload[-1]:
                                parsed = struct.unpack(packet_format, payload)
                                vx = parsed[1]
                                wz = parsed[2]
                                imu2_qw, imu2_qx, imu2_qy, imu2_qz = parsed[7:11]
                                
                                now = time.time()
                                dt = now - self.last_update_time
                                self.last_update_time = now
                                
                                self.current_vx = vx
                                self.current_wz = wz
                                self.odom.set_imu_quaternion(imu2_qw, imu2_qx, imu2_qy, imu2_qz)
                                self.odom.update(vx=vx, dt=dt)
                                del buffer[:52]
                            else:
                                del buffer[0:1]
                        else:
                            del buffer[0:1]
                except Exception as e:
                    print(f"串口读取异常: {e}")
                    time.sleep(0.5)

        t = threading.Thread(target=serial_loop, daemon=True)
        t.start()

    def _start_sim(self):
        print("💡 模拟测试模式已启用（无物理底盘连接，输入模拟驱动线速度 0.2m/s 进行测试）")
        def sim_loop():
            while self.is_running:
                time.sleep(0.05)
                # 若处于模拟模式，可保持上一状态

        t = threading.Thread(target=sim_loop, daemon=True)
        t.start()

    def _interactive_loop(self):
        with NonBlockingKeyboard() as kbd:
            last_print_time = 0.0
            while self.is_running:
                now = time.time()
                
                # 检查按键输入
                char = kbd.get_char()
                if char:
                    if char.lower() == 'q':
                        print("\n🛑 退出手动推车测试。")
                        self.is_running = False
                        break
                    elif char.lower() == 'r':
                        self.odom.reset_pose(0.0, 0.0, 0.0)
                        print("\n🔄 [RESET] 坐标已重置至原点 (0.000, 0.000, 0.0°)")
                    elif char.lower() == 'c':
                        self.odom.reset_distance()
                        print("\n🧹 [RESET] 累计里程统计已清零")
                    elif char in ['1', '2', '3', '4', '5', '6']:
                        pid = int(char)
                        loc = get_location(pid)
                        if loc:
                            self.odom.reset_pose(loc[0], loc[1], loc[2])
                            print(f"\n📍 [SET] 坐标已重新校准至 {pid}号位 ({loc[0]}, {loc[1]}, {loc[2]}°)")

                # 每 100ms 刷新一次终端状态 display
                if now - last_print_time >= 0.1:
                    last_print_time = now
                    self._print_status_ui()
                
                time.sleep(0.02)

    def _print_status_ui(self):
        x, y, theta_deg = self.odom.get_pose()
        summary = self.odom.get_summary()
        
        # 寻找最近的作业点位
        min_dist = float('inf')
        nearest_id = None
        for pid, (lx, ly, lth) in LOCATION_TABLE.items():
            dist, _, _ = calc_distance_and_heading(x, y, lx, ly)
            if dist < min_dist:
                min_dist = dist
                nearest_id = pid

        sys.stdout.write(
            f"\r⏱️ [实时死算位姿] X: {x:8.3f} m | Y: {y:8.3f} m | Theta: {theta_deg:6.1f}° "
            f"| 线速度: {self.current_vx:5.2f} m/s | 累计里程: {summary['total_distance']:6.2f} m "
            f"| 最近点: {nearest_id} (距{min_dist:4.2f}m)    "
        )
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description="Ackermann Odometry Manual Test Node")
    parser.add_argument('--mode', type=str, default='sim', choices=['ros2', 'serial', 'sim'],
                        help="运行模式: ros2, serial, sim")
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help="底盘串口设备号")
    parser.add_argument('--baud', type=int, default=460800, help="串口波特率")
    args = parser.parse_args()

    runner = OdomManualTestRunner(mode=args.mode, serial_port=args.port, baud_rate=args.baud)
    try:
        runner.start()
    except KeyboardInterrupt:
        print("\n用户中断，已退出。")


if __name__ == '__main__':
    main()
