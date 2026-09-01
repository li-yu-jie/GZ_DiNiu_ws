#!/usr/bin/env python3
# move_turn_test.py — 前进指定距离 + 原地转向指定角度的底盘测试脚本
#
# 用途：验证底盘运动学与里程计，例：
#   前进 1.0 m：          ros2 run ... 或 python3 move_turn_test.py --distance 1.0
#   原地左转 90°：        python3 move_turn_test.py --distance 0 --angle 90
#   先前进 0.5m 再右转 45°：python3 move_turn_test.py --distance 0.5 --angle -45
#
# 原理：
#   - 发 /cmd_vel (Twist)，20Hz（底盘看门狗 0.2s，必须 >5Hz 持续发）。
#   - 闭环反馈用里程计（默认 /wheel_odom，diuniu_base 无条件发布；
#     导航栈在线时可 --odom-topic /odom 用 FAST-LIO/EKF 里程计）。
#   - 前进段：相对起点位移达到目标距离即停；接近目标时线性减速防过冲。
#   - 转向段：/cmd_vel 走 Nav2 通道（allow_pure_rotation=True），允许 v=0 原地自转，
#     yaw 增量达到目标角度即停。正=左转(CCW)，负=右转(CW)。
#
# 注意：
#   - 不要与 Nav2 / 手柄同时发指令（手柄活跃期底盘仲裁会忽略本脚本，属预期）。
#   - 三轮车模型：+x 为车头方向（雷达端），倒车传负 distance。
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    """四元数 → yaw（纯 math，rootfs 无 tf_transformations）。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    """归一化到 [-pi, pi)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MoveTurnTest(Node):
    def __init__(self, args):
        super().__init__('move_turn_test')
        self.odom_topic = args.odom_topic
        self.target_dist = args.distance          # m，可正可负
        self.target_yaw = math.radians(args.angle)  # rad，正左转负右转
        self.max_v = abs(args.speed)
        self.max_w = abs(args.turn_speed)
        self.no_ramp = args.no_ramp
        self.dist_tol = args.dist_tol
        self.yaw_tol = math.radians(args.angle_tol)

        # 阶段: wait_odom -> drive -> turn -> done
        self.phase = 'wait_odom'
        if abs(self.target_dist) < 1e-6:
            self.phase_after_odom = 'turn' if abs(self.target_yaw) > 1e-6 else 'done'
        else:
            self.phase_after_odom = 'drive'

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.latest_yaw = None

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(
            f'目标: 前进 {self.target_dist:+.2f} m, 转向 {math.degrees(self.target_yaw):+.1f}°, '
            f'反馈源 {self.odom_topic}, 等待里程计...')

    def odom_cb(self, msg: Odometry):
        self.start_x = msg.pose.pose.position.x if self.start_x is None else self.start_x
        self.start_y = msg.pose.pose.position.y if self.start_y is None else self.start_y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.latest_yaw = yaw
        if self.start_yaw is None:
            self.start_yaw = yaw
            self.cur_x = msg.pose.pose.position.x
            self.cur_y = msg.pose.pose.position.y
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def dist_done(self):
        return math.hypot(self.cur_x - self.start_x, self.cur_y - self.start_y)

    def yaw_done(self):
        return wrap_angle(self.latest_yaw - self.start_yaw)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def control_loop(self):
        if self.phase == 'wait_odom':
            if self.start_yaw is not None:
                self.phase = self.phase_after_odom
                self.get_logger().info(f'里程计已锁定，进入阶段: {self.phase}')
            return

        if self.phase == 'drive':
            err = abs(self.target_dist) - self.dist_done()
            if err <= self.dist_tol:
                self.send(0.0, 0.0)
                self.get_logger().info(
                    f'✅ 前进完成: 里程计位移 {self.dist_done():.3f} m (目标 {self.target_dist:+.2f})'
                    f' ← 请卷尺实测实际距离，比例系数 = 实测/里程计')
                self.phase = 'turn' if abs(self.target_yaw) > 1e-6 else 'done'
                # 转向段以当前 yaw 重新取基线
                self.start_yaw = self.latest_yaw
                return
            direction = 1.0 if self.target_dist > 0 else -1.0
            if self.no_ramp:
                # 匀速标定模式：全程恒定速度，排除低速段虚报干扰
                v = direction * self.max_v
            else:
                # 末段线性减速：剩余 0.3m 内按比例降速，保底 0.10 m/s
                # （0.05 m/s 级已进入静摩擦虚报区，实测里程计严重偏多，勿再调低）
                v = direction * max(0.10, self.max_v * min(1.0, err / 0.3))
            self.send(v, 0.0)
            return

        if self.phase == 'turn':
            err = abs(self.target_yaw) - abs(self.yaw_done())
            if err <= self.yaw_tol:
                self.send(0.0, 0.0)
                self.get_logger().info(
                    f'✅ 转向完成: 实际转角 {math.degrees(self.yaw_done()):+.1f}° '
                    f'(目标 {math.degrees(self.target_yaw):+.1f}°)')
                self.phase = 'done'
                return
            direction = 1.0 if self.target_yaw > 0 else -1.0
            # 末段线性减速：剩余 20° 内按比例降速，保底 0.1 rad/s
            w = direction * max(0.1, self.max_w * min(1.0, err / math.radians(20.0)))
            self.send(0.0, w)
            return

        if self.phase == 'done':
            self.send(0.0, 0.0)
            self.get_logger().info('全部阶段完成，退出。')
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='前进指定距离 + 转向指定角度')
    parser.add_argument('--distance', type=float, default=1.0, help='前进距离 m（负=倒车），0 跳过')
    parser.add_argument('--angle', type=float, default=0.0, help='转向角度 deg（正=左转），0 跳过')
    parser.add_argument('--speed', type=float, default=0.2, help='前进线速度 m/s')
    parser.add_argument('--turn-speed', type=float, default=0.5, help='转向角速度 rad/s')
    parser.add_argument('--dist-tol', type=float, default=0.02, help='距离容差 m')
    parser.add_argument('--no-ramp', action='store_true',
                        help='匀速标定模式：全程恒速无减速段（标定时排除低速虚报干扰）')
    parser.add_argument('--angle-tol', type=float, default=2.0, help='角度容差 deg')
    parser.add_argument('--odom-topic', default='/wheel_odom',
                        help='里程计反馈话题（导航栈在线可用 /odom）')
    args = parser.parse_args()

    rclpy.init()
    node = MoveTurnTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0)
        node.get_logger().warn('⚠️ 被中断，已发送停车指令')
    finally:
        if rclpy.ok():
            node.send(0.0, 0.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
