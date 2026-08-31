import rclpy, math, time, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

DUR = 30

class P(Node):
    def __init__(self):
        super().__init__("pose_drift")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.amcl = None
        self.scan_near = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "amcl", m), 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)
    def on_scan(self, m):
        r = np.array(m.ranges)
        r = r[np.isfinite(r)]
        if len(r):
            self.scan_near = r.min()

def yaw_of(q):
    return math.degrees(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

rclpy.init()
n = P()
print(f"采样 {DUR}s（车必须静止）...")
rows = []
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.3)
    t = time.time() - t0
    amcl = odom = None
    if n.amcl is not None:
        p = n.amcl.pose.pose
        amcl = (p.position.x, p.position.y, yaw_of(p.orientation))
    try:
        tr = n.tf.lookup_transform("odom", "base_link", rclpy.time.Time())
        odom = (tr.transform.translation.x, tr.transform.translation.y,
                yaw_of(tr.transform.rotation))
    except Exception:
        pass
    if amcl or odom:
        rows.append((t, amcl, odom, n.scan_near))
    time.sleep(0.45)

print("\n t(s) | AMCL x,y,yaw | odom x,y,yaw | scan最近")
for t, a, o, sn in rows[::4]:
    fa = f"{a[0]:7.2f},{a[1]:7.2f},{a[2]:7.1f}" if a else "      ---      "
    fo = f"{o[0]:7.3f},{o[1]:7.3f},{o[2]:6.2f}" if o else "     ---     "
    fs = f"{sn:5.2f}" if sn else " --- "
    print(f"{t:5.1f} | {fa} | {fo} | {fs}")

def drift(key, idx):
    vals = [(t, v[idx]) for t, v in [(r[0], r[key]) for r in rows] if v is not None]
    if len(vals) < 2:
        return None
    arr = np.array([v[1] for v in vals])
    return arr[0], arr[-1], arr.max(0) - arr.min(0)

for name, key in [("AMCL", 1), ("odom", 2)]:
    d = drift(key, 0)
    if d:
        s, e, rng = d
        print(f"\n{name}: 起点 ({s[0]:.2f},{s[1]:.2f},{s[2]:.1f}°) 终点 ({e[0]:.2f},{e[1]:.2f},{e[2]:.1f}°)")
        print(f"{name}: 全程范围 Δx={rng[0]:.2f}m Δy={rng[1]:.2f}m Δyaw={rng[2]:.1f}°")
rclpy.shutdown()
