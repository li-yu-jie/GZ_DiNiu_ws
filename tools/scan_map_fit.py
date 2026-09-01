import os, rclpy, math, time, yaml, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.ndimage import distance_transform_edt

MAP_YAML = os.environ.get(
    "DIUNIU_MAP_YAML",
    os.path.join(os.environ.get("DIUNIU_WS", os.path.expanduser("~/GZ_DiNiu_ws")),
                 "src/diuniu_nav/maps/map.yaml"))

class P(Node):
    def __init__(self):
        super().__init__("scan_map_fit")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.pose = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "pose", m), 10)

rclpy.init()
n = P()
t0 = time.time()
while (n.scan is None or n.pose is None) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.5)
if n.scan is None or n.pose is None:
    print("超时: scan 或 amcl_pose 未到")
    raise SystemExit(1)

# 地图 + 距离场
with open(MAP_YAML) as f:
    m = yaml.safe_load(f)
p = MAP_YAML.rsplit("/", 1)[0] + "/" + m["image"]
with open(p, "rb") as f:
    assert f.readline().strip() == b"P5"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split()); int(f.readline())
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)
res = m["resolution"]; ox, oy = m["origin"][0], m["origin"][1]
occ = data < 128
edt = distance_transform_edt(~occ) * res  # 每个空格到最近占用的距离(米)

s = n.scan
pp = n.pose.pose.pose
px, py = pp.position.x, pp.position.y
pyaw = math.atan2(2 * pp.orientation.w * pp.orientation.z, 1 - 2 * pp.orientation.z ** 2)
rng = np.array(s.ranges)
ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
ok = np.isfinite(rng) & (rng > 0.2) & (rng < 12.0)
rng, ang = rng[ok], ang[ok]
print(f"AMCL位姿 ({px:.2f},{py:.2f},{math.degrees(pyaw):.1f}°)  有效beam={len(rng)}")

def score(yaw_off_deg):
    a = ang + pyaw + math.radians(yaw_off_deg)
    ex = px + rng * np.cos(a)
    ey = py + rng * np.sin(a)
    ix = ((ex - ox) / res - 0.5).astype(int)
    iy = (H - 1 - ((ey - oy) / res - 0.5)).astype(int)
    good = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    d = np.where(good, edt[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)], 3.0)
    return d.mean(), (d < 0.15).mean()

print("yaw偏移  平均落点离墙(m)  贴墙率(<0.15m)")
best = None
for off in range(-180, 181, 5):
    mean_d, hit = score(off)
    mark = ""
    if best is None or mean_d < best[1]:
        best = (off, mean_d, hit)
    if abs(off) <= 10 or off % 30 == 0:
        mark = " <<<" if abs(off) <= 0 else ""
    print(f"  {off:+4d}°   {mean_d:.3f}          {hit*100:.0f}%{mark}")
print(f"\n最佳: 偏移 {best[0]:+d}°  平均离墙 {best[1]:.3f}m  贴墙率 {best[2]*100:.0f}%")
rclpy.shutdown()
