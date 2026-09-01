#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车直接走指定坐标点名演示脚本 (run_move.py)
==============================================
使用说明：
  1. 终端 1 启动底盘驱动:
     ros2 launch diuniu_nav diuniu_nav_all.launch.py
  2. 终端 2 运行本脚本驱动小车前往指定坐标点名:
     python3 Path/run_move.py --point point_7 --speed 0.4 --mode forward
"""

import sys
import os
import time
import argparse
import threading

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from Path.ackermann_odometry import AckermannOdometry
    from Path.drive_control import DirectMoveController
    from Path.location_table import get_location
except ModuleNotFoundError:
    from ackermann_odometry import AckermannOdometry
    from drive_control import DirectMoveController
    from location_table import get_location


def run_chassis_move(point_name: str, speed: float = 0.4, mode: str = "forward"):
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
    except ImportError:
        print("❌ 未检测到 ROS 2 环境 (rclpy)！请先在终端中运行 `source install/setup.bash`。")
        return

    if not rclpy.ok():
        rclpy.init()

    node = Node('chassis_move_runner')
    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # 实例化里程计死算对象
    odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)

    # 订阅 IMU 与轮速里程计
    def imu_cb(msg):
        qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        odom.set_imu_quaternion(qw, qx, qy, qz)

    # 通过消息头时间戳计算真实 dt，消除时间抖动误差
    last_time = [None]
    def wheel_cb(msg):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if last_time[0] is not None:
            dt = curr_time - last_time[0]
            if 0.0 < dt < 1.0:
                odom.update(vx=msg.twist.twist.linear.x, dt=dt)
        else:
            # 首帧默认 dt=0.05
            odom.update(vx=msg.twist.twist.linear.x, dt=0.05)
        last_time[0] = curr_time

    node.create_subscription(Imu, '/imu/data', imu_cb, 10)
    node.create_subscription(Odometry, '/wheel_odom', wheel_cb, 10)

    # 启动后台 ROS 2 接收线程
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    # 速度指令发布包装函数
    def pub_cmd(vx, wz, vz_lift):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        msg.linear.z = float(vz_lift)
        cmd_pub.publish(msg)

    # 实例化控制器
    controller = DirectMoveController(odom, cmd_vel_pub_func=pub_cmd, dist_tolerance=0.20)

    # 稍微等待 ROS 2 传感器数据建立
    time.sleep(0.5)

    print(f"\n🚀 [准备就绪] 发起底盘运动 -> 点名: '{point_name}', 速度: {speed}m/s, 方向: {mode}")
    success = controller.move_to_point(point_name, speed=speed, mode=mode)

    if success:
        print(f"🎉 [成功] 小车已精准抵达点名 '{point_name}'！")
    else:
        print(f"⚠️ [失败/超时] 移动中断。")

    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="让底盘驱动走指定坐标点名")
    parser.add_argument('--point', type=str, default='a', help="要前往的坐标点名 (如 point_7, 1, 2, 3, parking)")
    parser.add_argument('--speed', type=float, default=0.4, help="运行线速度 (m/s)")
    parser.add_argument('--mode', type=str, default='forward', choices=['forward', 'reverse'], help="前进还是倒车")
    args = parser.parse_args()

    run_chassis_move(point_name=args.point, speed=args.speed, mode=args.mode)


if __name__ == '__main__':
    main()
