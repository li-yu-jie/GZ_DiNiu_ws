import rclpy, math, time
import numpy as np
from nav_msgs.msg import Odometry
from common import scan_xy, subscribe_scan

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
    rs = []
    for x, y in scan_xy(m):
        if abs(math.atan2(y, x)) <= math.radians(30):
            r = math.hypot(x, y)
            if r > m.range_min:
                rs.append(r)
    if rs:
        state["front_min"] = min(rs)
        state["front_n"] = len(rs)

node.create_subscription(Odometry, "/odom", odom_cb, 10)
subscribe_scan(node, "/scan", scan_cb)

print(" t  | FL-roll | FL-pitch | 前方±30°最近点 | 前方点数")
t0 = time.time()
while time.time() - t0 < 25:
    rclpy.spin_once(node, timeout_sec=0.05)
    t = time.time() - t0
    def f(v): return ("%7.2f" % v) if v is not None else "    ---"
    print("%4.1f | %s | %s | %s | %d" % (t, f(state["roll"]), f(state["pitch"]), f(state["front_min"]), state["front_n"]))
    time.sleep(0.3)
node.destroy_node(); rclpy.shutdown()
