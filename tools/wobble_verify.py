import rclpy, math, time
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 晃动测试：第 3~18 秒请用手轻摇雷达支架
# 对比指标：补偿后 /scan 在"正前方开阔区"(±30°) 的最近点距离是否稳定
rclpy.init()
node = rclpy.create_node("wobble_verify")
state = {"roll": None, "pitch": None, "front_min": None, "front_n": 0}

def odom_cb(m):
    q = m.pose.pose.orientation
    sinr = 2*(q.w*q.x + q.y*q.z)
    cosr = 1 - 2*(q.x*q.x + q.y*q.y)
    state["roll"] = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
    state["pitch"] = math.degrees(math.asin(sinp))

def scan_cb(m):
    # 正前方 ±30° 窗口(雷达朝车头方向, base_link 系 angle 0 = 车头)
    n_half = int(math.radians(30) / m.angle_increment)
    mid = len(m.ranges) // 2
    # 找到 angle=0 的下标
    i0 = int((0.0 - m.angle_min) / m.angle_increment)
    lo, hi = max(0, i0 - n_half), min(len(m.ranges), i0 + n_half)
    rs = [r for r in m.ranges[lo:hi] if math.isfinite(r) and r > m.range_min]
    if rs:
        state["front_min"] = min(rs)
        state["front_n"] = len(rs)

node.create_subscription(Odometry, "/odom", odom_cb, 10)
qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, "/scan", scan_cb, qos)

print(" t  | FL-roll | FL-pitch | 前方±30°最近点 | 前方点数")
t0 = time.time()
while time.time() - t0 < 25:
    rclpy.spin_once(node, timeout_sec=0.05)
    t = time.time() - t0
    def f(v): return ("%7.2f" % v) if v is not None else "    ---"
    print("%4.1f | %s | %s | %s | %d" % (t, f(state["roll"]), f(state["pitch"]), f(state["front_min"]), state["front_n"]))
    time.sleep(0.3)
node.destroy_node(); rclpy.shutdown()
