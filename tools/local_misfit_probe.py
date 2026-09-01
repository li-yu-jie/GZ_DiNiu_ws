import os, rclpy, math, time, yaml, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer, TransformListener

MAP_YAML = os.environ.get(
    "DIUNIU_MAP_YAML",
    os.path.join(os.environ.get("DIUNIU_WS", os.path.expanduser("~/GZ_DiNiu_ws")),
                 "src/diuniu_nav/maps/map.yaml"))

class P(Node):
    def __init__(self):
        super().__init__("local_refine")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos)
        self.tf = Buffer()
        TransformListener(self.tf, self)

rclpy.init()
n = P()
t0 = time.time()
tr = None
while time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.5)
    if n.scan is not None:
        try:
            tr = n.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            break
        except Exception:
            continue
if n.scan is None:
    print("超时: /scan 未到")
    raise SystemExit(1)
if tr is None:
    print("超时: map->base_link TF 20s 未出现（AMCL 未在广播）")
    raise SystemExit(1)

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
edt = distance_transform_edt(~(data < 128)) * res

s = n.scan
px, py = tr.transform.translation.x, tr.transform.translation.y
qz, qw = tr.transform.rotation.z, tr.transform.rotation.w
pyaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
rng = np.array(s.ranges)
ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
ok = np.isfinite(rng) & (rng > 0.2) & (rng < 10.0)
rng, ang = rng[ok], ang[ok]
print(f"AMCL位姿 ({px:.2f},{pyaw and py:.2f},{math.degrees(pyaw):.1f}°)  有效beam={len(rng)}")

def score(x, y, yaw):
    a = ang + yaw
    ex = x + rng * np.cos(a)
    ey = y + rng * np.sin(a)
    ix = ((ex - ox) / res - 0.5).astype(np.int32)
    iy = (H - 1 - ((ey - oy) / res - 0.5)).astype(np.int32)
    good = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    d = np.where(good, edt[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)], 2.0)
    return d.mean(), (d < 0.15).mean()

m0, h0 = score(px, py, pyaw)
print(f"当前位姿直接评分: 平均={m0:.3f}m 贴墙率={h0*100:.0f}%")

best = (m0, 0.0, 0.0, 0.0, h0)
for dx in np.arange(-0.8, 0.81, 0.1):
    for dy in np.arange(-0.8, 0.81, 0.1):
        for dya in range(-8, 9, 2):
            mm, hh = score(px + dx, py + dy, pyaw + math.radians(dya))
            if mm < best[0]:
                best = (mm, dx, dy, dya, hh)
print(f"局部最优: 偏移 dx={best[1]:+.2f}m dy={best[2]:+.2f}m dyaw={best[3]:+d}° → 平均={best[0]:.3f}m 贴墙率={best[4]*100:.0f}%")
rclpy.shutdown()
