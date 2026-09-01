#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
执行您的专属顺序避障路径脚本 (run_sequential_path.py)
======================================================
运行路线: tingche -> tingche_1 -> cang_1 -> cang_1_1
使用说明：
  1. 确认底盘驱动节点已启动:
     ros2 launch diuniu_nav diuniu_nav_all.launch.py
  2. 运行本脚本：
     python3 Path/run_sequential_path.py
"""

import sys
import os
import time
import threading

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from Path.ackermann_odometry import AckermannOdometry
    from Path.drive_control import DirectMoveController
except ModuleNotFoundError:
    from ackermann_odometry import AckermannOdometry
    from drive_control import DirectMoveController


def main():
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
    except ImportError:
        print("❌ 未检测到 ROS 2 环境 (rclpy)！请先运行 `source install/setup.bash`。")
        return

    if not rclpy.ok():
        rclpy.init()

    node = Node('run_sequential_path_node')
    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # 1. 实例化绝对里程计 (scale_factor=0.936：FAST-LIO 雷达真值标定的轮速比例系数)
    odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0, scale_factor=0.936)

    # 2. 订阅物理反馈
    node.create_subscription(Imu, '/imu/data', 
                             lambda msg: odom.set_imu_quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z), 10)

    last_time = [None]
    def wheel_cb(msg):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if last_time[0] is not None:
            dt = curr_time - last_time[0]
            if 0.0 < dt < 1.0:
                odom.update(vx=msg.twist.twist.linear.x, dt=dt)
        else:
            odom.update(vx=msg.twist.twist.linear.x, dt=0.05)
        last_time[0] = curr_time

    node.create_subscription(Odometry, '/wheel_odom', wheel_cb, 10)

    # 启动后台 ROS 2 spin
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    # 3. 速度下发包装函数
    def pub_cmd(vx, wz, vz_lift):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        msg.linear.z = float(vz_lift)
        cmd_pub.publish(msg)

    # 4. 实例化控制器
    controller = DirectMoveController(odom, cmd_vel_pub_func=pub_cmd, dist_tolerance=0.20, angle_tolerance_deg=5.0)

    time.sleep(0.5)

    # 5. 您自定义的顺序路线列表
    # 格式: (点名, 速度, 前进还是倒车)
    my_sequential_path = [
        ("tingche", 0.4, "forward"),
        ("tingche_1", 0.4, "forward"),
        ("cang_1", 0.4, "forward"),
        ("cang_1_1", 0.3, "reverse")
    ]

    print(f"\n🚙 [路径启动] 开始依次前往以下航点: {[' -> '.join([wp[0] for wp in my_sequential_path])]}...")
    
    # 运行顺序路径
    process = None

    def kill_align_process():
        """整组杀掉 AprilTag 视觉链（bash/ros2 launch/tag_align/相机节点全在一个进程组）。

        必须 start_new_session + killpg：只 terminate() bash 包装会把容器内
        ros2 launch 及其子节点留成孤儿，残留的 tag_align 会继续向 /cmd_vel
        发指令与主脚本打架（ DONE 态发零速、被拖离后再武装主动倒车）。
        """
        nonlocal process
        if process is None or process.poll() is not None:
            process = None
            return
        import signal
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except Exception:
                pass
        process = None

    try:
        success = controller.move_path_by_names(my_sequential_path)
    except KeyboardInterrupt:
        print("\n🛑 [人工中断] 收到 Ctrl+C，立即停车并退出！")
        controller.stop()
        node.destroy_node()
        rclpy.shutdown()
        return

    if success:
        print("\n🎉 [顺利完成] 小车已成功跑完您规划的所有航点！开始进入视觉矫正流程...")

        try:
            # 1. 直接启动 AprilTag 视觉对中 (对齐完成前不下降货叉)
            print("🚀 [视觉伺服] 正在拉起 AprilTag 视觉对位节点...")
            import subprocess
            import select
            cmd = ["bash", "/home/y/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh", "align:=true"]

            # 启动子进程（独立进程组，便于整组清理），实时读取输出
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, bufsize=1, start_new_session=True)

            aligned = False
            start_align_time = time.time()

            # 设定 60 秒对位超时限时 (select 轮询读取，readline 阻塞不会架空超时)
            while time.time() - start_align_time < 60.0:
                if process.poll() is not None:
                    break
                r, _, _ = select.select([process.stdout], [], [], 1.0)
                if not r:
                    continue
                line = process.stdout.readline()
                if not line:
                    continue

                # 实时打印 AprilTag 节点日志
                sys.stdout.write(f"[AprilTag] {line}")
                sys.stdout.flush()

                # 检测是否成功输出 "已到位"
                if "已到位" in line:
                    print("\n🎉 [视觉伺服] 检测到对位成功完成标志 (已到位)！")
                    aligned = True
                    break

            # ★ 无论成败，第一时间整组杀掉视觉链：tag_align DONE 态会持续
            # 20Hz 发零速，且车被倒开 1.2m 后会自动再武装主动倒车追码——
            # 不杀会与下方所有动作在 /cmd_vel 上打架（"走-停-走"抖动根因）
            kill_align_process()

            if aligned:
                # 2. 视觉对齐完成后，下降货叉 (持续发送 5s 指令，每 50ms 发送一次以防看门狗超时)
                print("📦 [货叉动作] 视觉矫正已完成，开始发送降低货叉指令 (持续 5.0 秒，发送间隔 50ms)...")
                start_t = time.time()
                while time.time() - start_t < 5.0:
                    pub_cmd(0.0, 0.0, -1.0)
                    time.sleep(0.05)
                pub_cmd(0.0, 0.0, 0.0)
                print("✅ [货叉动作] 降低指令发送完毕，货叉已降至最低。")

                # 3. 后退 1.2m 拔叉/插叉 (通过高精度里程计闭环控制距离，带超时保护)
                print("🚙 [插车插取] 开始直线后退 1.2m 以将货叉插入托盘...")
                import math
                start_x, start_y, _ = odom.get_pose()
                backed_dist = 0.0
                last_bk_print = 0.0
                bk_start = time.time()
                bk_ok = True

                # 以 -0.3 m/s 的倒车线速度向后退
                while backed_dist < 1.20:
                    if time.time() - bk_start > 12.0:
                        print("⚠️ [插车插取] 倒车 12s 超时（里程计停更/车轮受阻），紧急停车并中止后续动作！")
                        bk_ok = False
                        break
                    pub_cmd(-0.3, 0.0, 0.0)
                    time.sleep(0.05)
                    curr_x, curr_y, _ = odom.get_pose()
                    backed_dist = math.hypot(curr_x - start_x, curr_y - start_y)

                    now_t = time.time()
                    if now_t - last_bk_print >= 1.0:
                        last_bk_print = now_t
                        print(f"🚙 [插车插取] 当前已后退: {backed_dist:.2f}m / 1.20m")

                pub_cmd(0.0, 0.0, 0.0)

                if bk_ok:
                    print("✅ [插车插取] 已成功后退 1.20m，货叉已对准就位。")

                    # 4. 执行升起动作 (持续发送 4s 指令，每 50ms 发送一次以防看门狗超时)
                    print("📦 [货叉动作] 开始发送升起货叉指令 (持续 4.0 秒，发送间隔 50ms)...")
                    start_t = time.time()
                    while time.time() - start_t < 4.0:
                        pub_cmd(0.0, 0.0, 1.0)
                        time.sleep(0.05)
                    pub_cmd(0.0, 0.0, 0.0)
                    print("✅ [货叉动作] 升起指令发送完毕，货物插取成功！")

                    # 5. 插完后去 cang_1
                    print("🚙 [出库行驶] 货物插取完毕，正在将托盘货物拉回 cang_1 点...")
                    # 此时前轮应该从原位行驶至 cang_1 (X=1.8, Y=1.65, Theta=90.0)
                    ret_to_cang = controller.move_to_point("cang_1", speed=0.4, mode="forward")
                    if ret_to_cang:
                        print("🎉 [作业成功] 小车已成功插货并带货驶回 cang_1 点，流程顺利结束！")
                    else:
                        print("⚠️ [作业异常] 驶回 cang_1 点失败或超时。")
            else:
                print("❌ [视觉伺服] 视觉对位超时或子进程异常退出，未执行后续动作。")

        finally:
            # 异常/中断兜底：发零速 + 确保视觉链死透，绝不留残留 /cmd_vel 发布者
            pub_cmd(0.0, 0.0, 0.0)
            kill_align_process()

        print("✅ [流程结束] 视觉精调与插取货流程已全部处理完毕。")
    else:
        print("\n⚠️ [任务中断] 移动未能全部完成，未进入视觉与插货流程。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
