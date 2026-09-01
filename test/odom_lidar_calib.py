#!/usr/bin/env python3
# odom_lidar_calib.py — 以 FAST-LIO 雷达里程计为真值标定轮式里程计 vx
#
# 原理：
#   - /wheel_odom：被标定对象（下位机按轮径标称上报，含系统误差）
#   - /odom (FAST-LIO 雷达惯性里程计, 10Hz)：真值参考，短距 1~2m 精度 cm 级
#   - /imu/data (BNO085)：航向参考，报告每次运行的跑偏量
#   每次运行：恒速发 /cmd_vel 直到 wheel_odom 位移达标 → 停车 → 等车身静止
#   → 记录 轮式位移 W 与 雷达位移 L，k = L/W 即真实比例。
#
# 安全：每次运行前用 /cloud_registered_body 检查行进走廊（侧向 ±0.45m、
#   前方 dist+0.8m）有无障碍物点簇，有则中止该次运行。
#
# 用法（容器内）：
#   source /opt/ros/humble/setup.bash && source install/setup.bash
#   python3 test/odom_lidar_calib.py                 # 默认 0.2/0.4/0.6 × 1m/2m 矩阵
#   python3 test/odom_lidar_calib.py --runs 0.4:2.0  # 单次：速度0.4 距离2m
import argparse
import math
import struct
import sys
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


class Calib(Node):
    def __init__(self):
        super().__init__('odom_lidar_calib')
        self.wheel = None   # (x, y) 最新轮式里程计
        self.lio = None     # (x, y, vx) 最新雷达里程计
        self.imu_yaw = None
        self.cloud = None   # 最新点云（body 系）
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_cb, 10)
        self.create_subscription(Odometry, '/odom', self.lio_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.cloud_cb, 10)

    def wheel_cb(self, m):
        self.wheel = (m.pose.pose.position.x, m.pose.pose.position.y)

    def lio_cb(self, m):
        self.lio = (m.pose.pose.position.x, m.pose.pose.position.y,
                    m.twist.twist.linear.x, m.twist.twist.linear.y)

    def imu_cb(self, m):
        self.imu_yaw = yaw_from_quat(m.orientation)

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
            if self.wheel and self.lio and self.imu_yaw is not None and self.cloud:
                return True
        return False

    def corridor_clear(self, direction, dist):
        """检查行进走廊：body 系前方 direction*[0.3, dist+0.8]，|y|<0.45，
        z∈[-0.45, 1.0]（雷达系，地面 z≈-0.66，以上算障碍）。"""
        m = self.cloud
        if m is None:
            return False, -1
        # 手工解析 PointCloud2（假定 x,y,z 为 float32，PointXYZI 布局）
        offs = {}
        for f in m.fields:
            offs[f.name] = f.offset
        if not all(k in offs for k in ('x', 'y', 'z')):
            return False, -2
        ox, oy, oz = offs['x'], offs['y'], offs['z']
        step = m.point_step
        n = m.width * m.height
        data = m.data
        cnt = 0
        for i in range(n):
            base = i * step
            x = struct.unpack_from('<f', data, base + ox)[0]
            y = struct.unpack_from('<f', data, base + oy)[0]
            z = struct.unpack_from('<f', data, base + oz)[0]
            fwd = x * direction
            if 0.3 < fwd < dist + 0.8 and abs(y) < 0.45 and -0.45 < z < 1.0:
                cnt += 1
        return cnt < 30, cnt

    def send(self, v):
        msg = Twist()
        msg.linear.x = v
        self.pub.publish(msg)

    def run_once(self, speed, dist, direction):
        ok, cnt = self.corridor_clear(direction, dist)
        if not ok:
            self.get_logger().error(
                f'⛔ 走廊有障碍（{cnt} 点），中止本次: v={speed} d={dist} dir={direction}')
            return None

        w0 = self.wheel
        l0 = self.lio
        yaw0 = self.imu_yaw
        v = speed * direction
        self.get_logger().info(f'▶ 开始: v={v:+.2f} m/s 目标 {dist:.1f} m')

        # 恒速行驶直到轮式位移达标（20Hz 发令，看门狗 0.2s）
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            self.send(v)
            wd = math.hypot(self.wheel[0] - w0[0], self.wheel[1] - w0[1])
            if wd >= dist:
                break
            if time.time() - t0 > dist / speed * 3 + 10:
                self.get_logger().error('⛔ 行驶超时，停车中止')
                self.send(0.0)
                return None
        self.send(0.0)

        # 等车身静止（雷达里程计速度 <2cm/s 持续 0.5s，超时 6s）
        t_stop = time.time()
        t_still = None
        while time.time() - t_stop < 6.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            lv = math.hypot(self.lio[2], self.lio[3])
            if lv < 0.02:
                t_still = t_still or time.time()
                if time.time() - t_still > 0.5:
                    break
            else:
                t_still = None
        self.send(0.0)

        W = math.hypot(self.wheel[0] - w0[0], self.wheel[1] - w0[1])
        L = math.hypot(self.lio[0] - l0[0], self.lio[1] - l0[1])
        dyaw = math.degrees(wrap_angle(self.imu_yaw - yaw0))
        k = L / W if W > 0.01 else float('nan')
        return dict(v=speed, dist=dist, dir=direction, W=W, L=L, k=k, dyaw=dyaw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='*', default=None,
                    help='形如 0.4:2.0 的 速度:距离 列表；缺省跑完整矩阵')
    args = ap.parse_args()

    if args.runs:
        matrix = []
        for r in args.runs:
            v, d = r.split(':')
            matrix.append((float(v), float(d)))
    else:
        matrix = [(0.2, 1.0), (0.2, 2.0), (0.4, 1.0),
                  (0.4, 2.0), (0.6, 1.0), (0.6, 2.0)]

    rclpy.init()
    node = Calib()
    if not node.wait_ready():
        node.get_logger().error('⛔ 话题未就绪（需要 /wheel_odom /odom /imu/data /cloud_registered_body）')
        return

    results = []
    # 方向交替：前进后倒车返回，始终在活动范围内
    for i, (v, d) in enumerate(matrix):
        direction = 1 if i % 2 == 0 else -1
        r = node.run_once(v, d, direction)
        if r:
            results.append(r)
            print(f'[结果] v={r["v"]:.1f} d={r["dist"]:.1f} {"前进" if r["dir"]>0 else "倒车"}: '
                  f'轮式={r["W"]:.3f}m 雷达={r["L"]:.3f}m k={r["k"]:.4f} '
                  f'跑偏={r["dyaw"]:+.2f}°', flush=True)
        node.spin_for(2.0)  # 间隔，停稳再下一次

    node.send(0.0)
    print('\n===== 汇总 =====')
    for r in results:
        print(f'v={r["v"]:.1f} d={r["dist"]:.1f} dir={r["dir"]:+d} '
              f'W={r["W"]:.3f} L={r["L"]:.3f} k={r["k"]:.4f} dyaw={r["dyaw"]:+.2f}°')

    # 最小二乘拟合 k = scale - slip*|v|（分前进/倒车）
    for dirn, name in ((1, '前进'), (-1, '倒车')):
        pts = [(r['v'], r['k']) for r in results if r['dir'] == dirn]
        if len(pts) >= 2:
            n = len(pts)
            sv = sum(p[0] for p in pts)
            sk = sum(p[1] for p in pts)
            svv = sum(p[0] ** 2 for p in pts)
            svk = sum(p[0] * p[1] for p in pts)
            den = n * svv - sv * sv
            if abs(den) > 1e-9:
                slip = -(n * svk - sv * sk) / den
                scale = (sk + slip * sv) / n
                print(f'[{name}拟合] k(v) = {scale:.4f} - {slip:.4f}*|v|  '
                      f'→ odom_vx_scale={scale:.4f} odom_vx_slip={slip:.4f}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
