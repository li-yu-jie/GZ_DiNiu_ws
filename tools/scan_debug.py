#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

rclpy.init()
node = Node("scan_debug")

raw_scan = None
filt_scan = None
count = [0, 0]

def cb_scan(msg):
    global raw_scan
    raw_scan = msg
    count[0] += 1
def cb_filt(msg):
    global filt_scan
    filt_scan = msg
    count[1] += 1

node.create_subscription(LaserScan, "/scan", cb_scan, qos_profile_sensor_data)
node.create_subscription(LaserScan, "/scan_filtered", cb_filt, qos_profile_sensor_data)

start = node.get_clock().now()
while (node.get_clock().now() - start).nanoseconds < 3e9:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"scan msgs: {count[0]}, filtered msgs: {count[1]}")

def analyze(name, scan):
    if not scan:
        print(f"{name}: no data")
        return
    valid = []
    for i, r in enumerate(scan.ranges):
        if not math.isinf(r) and not math.isnan(r):
            ang = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(ang)
            y = r * math.sin(ang)
            valid.append((r, math.degrees(ang), x, y))
    print(f"=== {name} === valid={len(valid)}/{len(scan.ranges)}")
    valid.sort(key=lambda t: t[0])
    # 屏蔽盒（须与 laserscan_filter 参数/两个 launch 同步）：x∈[-1.65,1.65], y=±0.36
    for r_val, deg, x, y in valid[:20]:
        in_box = (-1.65 <= x <= 1.65 and -0.36 <= y <= 0.36)
        tag = "LEAK" if in_box else "OK"
        print(f"  r={r_val:.2f}m deg={deg:+6.1f} x={x:+5.2f} y={y:+5.2f} [{tag}]")

analyze("/scan", raw_scan)
analyze("/scan_filtered", filt_scan)
