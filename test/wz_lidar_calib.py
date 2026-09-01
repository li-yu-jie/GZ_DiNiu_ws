#!/usr/bin/env python3
# wz_lidar_calib.py — 以 FAST-LIO yaw + BNO085 yaw 双真值标定轮式里程计 wz
#
# 原理（与 odom_lidar_calib.py 对称）：
#   - /wheel_odom 的 th：被标定对象（下位机上报 wz 的积分，三轮车模型下
#     wz 由驱动轮速度+打角推算，含轮距/零位误差）
#   - /odom (FAST-LIO) yaw 增量：真值 1
#   - /imu/data (BNO085) yaw 增量：真值 2（绝对航向，与雷达互相印证）
#   每次运行：恒定角速度原地旋转直到轮式转角达标 → 停车 → 静止后记录
#   三方转角增量，k = 真值/轮式。
#   转角用逐帧 unwrap 累积（可跨 ±π，支持 360°+）。
#
# 安全：原地旋转前检查车身周围 0.5~2.2m 环带有无障碍物点簇（旋转扫掠
#   半径大，货叉端约 1.65m）。
#
# 用法（容器内）：
#   python3 test/wz_lidar_calib.py                    # 默认 0.5/1.0 rad/s × 180°/360°
#   python3 test/wz_lidar_calib.py --runs 0.8:180     # 单次：角速度0.8 转180°
import argparse
import math
import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class YawTracker:
    """逐帧 unwrap 累积转角，可跨 ±π。"""

    def __init__(self):
        self.last = None
        self.cum = 0.0

    def update(self, yaw):
        if self.last is not None:
            self.cum += wrap_angle(yaw - self.last)
        self.last = yaw
        return self.cum


class WzCalib(Node):
    def __init__(self, cmd_topic):
        super().__init__('wz_lidar_calib')
        self.wheel_yaw = YawTracker()
        self.lio_yaw = YawTracker()
        self.imu_yaw = YawTracker()
        self.lio_wz = 0.0
        self.cloud = None
        # Nav2 在线时发 /cmd_vel_nav（经 velocity_smoother + collision_monitor
        # 安全链到 /cmd_vel），避免与 CM 输出直接冲突；裸底盘时发 /cmd_vel
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.lio_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.cloud_cb, 10)

    def wheel_cb(self, m):
        self.wheel_yaw.update(yaw_from_quat(m.pose.pose.orientation))

    def lio_cb(self, m):
        self.lio_yaw.update(yaw_from_quat(m.pose.pose.orientation))
        self.lio_wz = m.twist.twist.angular.z

    def imu_cb(self, m):
        self.imu_yaw.update(yaw_from_quat(m.orientation))

    def cloud_cb(self, m):
        self.cloud = m

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.wheel_yaw.last is not None and self.lio_yaw.last is not None
                    and self.imu_yaw.last is not None and self.cloud):
                return True
        return False

    def surroundings_clear(self):
        """旋转扫掠检查：body 系 0.5~2.2m 环带、z∈[-0.45,1.0] 的障碍点数。"""
        m = self.cloud
        if m is None:
            return False, -1
        offs = {f.name: f.offset for f in m.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return False, -2
        ox, oy, oz = offs['x'], offs['y'], offs['z']
        step, n, data = m.point_step, m.width * m.height, m.data
        cnt = 0
        for i in range(n):
            base = i * step
            x = struct.unpack_from('<f', data, base + ox)[0]
            y = struct.unpack_from('<f', data, base + oy)[0]
            z = struct.unpack_from('<f', data, base + oz)[0]
            r = math.hypot(x, y)
            if 0.5 < r < 2.2 and -0.45 < z < 1.0:
                cnt += 1
        return cnt < 30, cnt

    def send(self, w):
        msg = Twist()
        msg.angular.z = w
        self.pub.publish(msg)

    def run_once(self, w_speed, angle_deg, direction):
        ok, cnt = self.surroundings_clear()
        if not ok:
            self.get_logger().error(f'⛔ 周围有障碍（{cnt} 点），中止本次旋转')
            return None

        w0, l0, i0 = self.wheel_yaw.cum, self.lio_yaw.cum, self.imu_yaw.cum
        target = math.radians(angle_deg)
        w = w_speed * direction
        self.get_logger().info(
            f'▶ 开始: w={w:+.2f} rad/s 目标 {angle_deg:.0f}° '
            f'({"逆时针" if direction > 0 else "顺时针"})')

        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            self.send(w)
            if abs(self.wheel_yaw.cum - w0) >= target:
                break
            if time.time() - t0 > target / w_speed * 3 + 10:
                self.get_logger().error('⛔ 旋转超时，停车中止')
                self.send(0.0)
                return None
        self.send(0.0)

        # 等静止（雷达角速度 <1°/s 持续 0.5s，超时 5s）
        t_stop, t_still = time.time(), None
        while time.time() - t_stop < 5.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            if abs(self.lio_wz) < math.radians(1.0):
                t_still = t_still or time.time()
                if time.time() - t_still > 0.5:
                    break
            else:
                t_still = None
        self.send(0.0)

        dW = self.wheel_yaw.cum - w0
        dL = self.lio_yaw.cum - l0
        dI = self.imu_yaw.cum - i0
        return dict(w=w_speed, ang=angle_deg, dir=direction,
                    dW=math.degrees(dW), dL=math.degrees(dL), dI=math.degrees(dI),
                    k_lio=dL / dW if abs(dW) > 1 else float('nan'),
                    k_imu=dI / dW if abs(dW) > 1 else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='*', default=None,
                    help='形如 0.8:180 的 角速度:角度 列表；缺省跑完整矩阵')
    ap.add_argument('--cmd-topic', default='/cmd_vel_nav',
                    help='速度指令话题：Nav2 在线用 /cmd_vel_nav（默认），裸底盘用 /cmd_vel')
    args = ap.parse_args()

    if args.runs:
        matrix = []
        for r in args.runs:
            w, a = r.split(':')
            matrix.append((float(w), float(a)))
    else:
        matrix = [(0.5, 180), (0.5, 360), (1.0, 180), (1.0, 360)]

    rclpy.init()
    node = WzCalib(args.cmd_topic)
    if not node.wait_ready():
        node.get_logger().error('⛔ 话题未就绪（需要 /wheel_odom /odom /imu/data /cloud_registered_body）')
        return

    results = []
    # 方向交替：CCW 后 CW 转回，避免线缆/朝向问题
    for i, (w, a) in enumerate(matrix):
        direction = 1 if i % 2 == 0 else -1
        r = node.run_once(w, a, direction)
        if r:
            results.append(r)
            print(f'[结果] w={r["w"]:.1f} {r["ang"]:.0f}° {"CCW" if r["dir"]>0 else "CW"}: '
                  f'轮式={r["dW"]:+.1f}° 雷达={r["dL"]:+.1f}° IMU={r["dI"]:+.1f}° '
                  f'k_雷达={r["k_lio"]:.4f} k_IMU={r["k_imu"]:.4f}', flush=True)
        node.spin_for(2.0)

    node.send(0.0)
    print('\n===== 汇总 =====')
    for r in results:
        print(f'w={r["w"]:.1f} {r["ang"]:.0f}° dir={r["dir"]:+d} '
              f'dW={r["dW"]:+.1f}° dL={r["dL"]:+.1f}° dI={r["dI"]:+.1f}° '
              f'k_lio={r["k_lio"]:.4f} k_imu={r["k_imu"]:.4f}')
    if results:
        # 合并转角比（比逐次平均更抗单次噪声）
        sW = sum(abs(r['dW']) for r in results)
        sL = sum(abs(r['dL']) for r in results)
        sI = sum(abs(r['dI']) for r in results)
        print(f'[合并] 轮式总转角={sW:.1f}° 雷达={sL:.1f}° IMU={sI:.1f}° '
              f'→ k_雷达={sL/sW:.4f} k_IMU={sI/sW:.4f}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
