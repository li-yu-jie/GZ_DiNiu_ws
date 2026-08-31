import rclpy, math, time, yaml, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt

MAP_YAML = "/home/y/GZ_DiNiu_ws/src/diuniu_nav/maps/map.yaml"

class P(Node):
    def __init__(self):
        super().__init__("scan_map_grid")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = None
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos)

rclpy.init()
n = P()
t0 = time.time()
while n.scan is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.5)
if n.scan is None:
    print("超时: /scan 未到")
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
occ = data < 128
edt = distance_transform_edt(~occ) * res

s = n.scan
rng = np.array(s.ranges)
ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
ok = np.isfinite(rng) & (rng > 0.2) & (rng < 10.0)
rng, ang = rng[ok], ang[ok]
print(f"有效beam={len(rng)}  地图 {W}x{H} res={res}")

# 自由空间 1.0m 网格
step = max(1, int(round(1.0 / res)))
free_y, free_x = np.nonzero(~occ)
keep = (free_x % step == 0) & (free_y % step == 0)
cx = ox + (free_x[keep] + 0.5) * res
cy = oy + (H - 1 - free_y[keep] + 0.5) * res
print(f"候选位置 {len(cx)} 个（1.0m 网格）")

cos_a, sin_a = np.cos(ang), np.sin(ang)
best = []
for yaw_deg in range(-180, 180, 3):
    yr = math.radians(yaw_deg)
    cyw, syw = math.cos(yr), math.sin(yr)
    dx = rng * (cos_a * cyw - sin_a * syw)
    dy = rng * (cos_a * syw + sin_a * cyw)
    ex = cx[:, None] + dx[None, :]
    ey = cy[:, None] + dy[None, :]
    ix = ((ex - ox) / res - 0.5).astype(np.int32)
    iy = (H - 1 - ((ey - oy) / res - 0.5)).astype(np.int32)
    good = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    d = np.where(good, edt[np.clip(iy, 0, H - 1), np.clip(ix, 0, W - 1)], 2.0)
    frac_ok = good.mean(axis=1)
    mean_d = d.mean(axis=1)
    valid = frac_ok > 0.7
    if not valid.any():
        continue
    i = np.argmin(np.where(valid, mean_d, 9e9))
    best.append((mean_d[i], cx[i], cy[i], yaw_deg))

best.sort()
print("\n=== 全局最佳 10 个位姿 ===")
for mean_d, x, y, yd in best[:10]:
    print(f"  ({x:7.2f},{y:7.2f}) yaw={yd:+5d}°  平均离墙={mean_d:.3f}m")
rclpy.shutdown()
