import rclpy, math, time
from nav_msgs.msg import Odometry
from common import scan_xy, subscribe_scan

rclpy.init()
node = rclpy.create_node("wobble_probe")
state = {"roll": None, "pitch": None, "min_r": None}

def odom_cb(m):
    q = m.pose.pose.orientation
    # roll, pitch from quaternion
    sinr = 2*(q.w*q.x + q.y*q.z)
    cosr = 1 - 2*(q.x*q.x + q.y*q.y)
    state["roll"] = math.degrees(math.atan2(sinr, cosr))
    sinp = 2*(q.w*q.y - q.z*q.x)
    sinp = max(-1.0, min(1.0, sinp))
    state["pitch"] = math.degrees(math.asin(sinp))

def scan_cb(m):
    rs = [r for r in (math.hypot(x, y) for x, y in scan_xy(m)) if r > m.range_min]
    if rs:
        state["min_r"] = min(rs)

node.create_subscription(Odometry, "/odom", odom_cb, 10)
subscribe_scan(node, "/scan_filtered", scan_cb)

print(" t  | FL-roll | FL-pitch | scan最近点   (请在第5~15秒间摇动支架)")
t0 = time.time()
while time.time() - t0 < 22:
    rclpy.spin_once(node, timeout_sec=0.05)
    t = time.time() - t0
    def f(v): return ("%7.2f" % v) if v is not None else "    ---"
    print("%4.1f | %s | %s | %s" % (t, f(state["roll"]), f(state["pitch"]), f(state["min_r"])))
    time.sleep(0.2)
node.destroy_node(); rclpy.shutdown()
