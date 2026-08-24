import rclpy, math, time
import numpy as np
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 12 扇区(每 30°)最近点监视：补偿生效时，摇晃支架不应改变任何扇区的距离
rclpy.init()
node = rclpy.create_node("sector_watch")
state = {"sectors": None, "stamp": 0}
NSEC = 12

def scan_cb(m):
    sec = [[] for _ in range(NSEC)]
    ang = m.angle_min
    for r in m.ranges:
        if math.isfinite(r) and r > m.range_min:
            a = ang
            while a > math.pi: a -= 2*math.pi
            while a < -math.pi: a += 2*math.pi
            i = int((a + math.pi) / (2*math.pi) * NSEC) % NSEC
            sec[i].append(r)
        ang += m.angle_increment
    state["sectors"] = [min(s) if s else float('inf') for s in sec]
    state["stamp"] += 1

qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, "/scan", scan_cb, qos)

print("扇区定义: 0=正后方(-180°) 3=左侧(-90°) 6=正前方(0°) 9=右侧(+90°)")
print("时间 | " + " | ".join("%5d" % (i*30-180+15) for i in range(NSEC)))
baseline = None
t0 = time.time()
last = -1
while time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.05)
    if state["stamp"] != last and state["sectors"]:
        last = state["stamp"]
        cur = state["sectors"]
        if baseline is None:
            baseline = cur
        cells = []
        for b, c in zip(baseline, cur):
            if math.isinf(c):
                cells.append("  ---")
            else:
                d = c - b if not math.isinf(b) else 0.0
                mark = "*" if abs(d) > 0.15 else " "
                cells.append("%4.2f%s" % (c, mark))
        print("%4.1f | %s" % (time.time()-t0, " ".join(cells)))
        time.sleep(0.25)
print("(* = 与首帧偏差 >15cm)")
node.destroy_node(); rclpy.shutdown()
