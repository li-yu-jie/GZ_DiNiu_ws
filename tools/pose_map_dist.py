import os, rclpy, math, time, yaml, numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

MAP_YAML = os.environ.get(
    "DIUNIU_MAP_YAML",
    os.path.join(os.environ.get("DIUNIU_WS", os.path.expanduser("~/GZ_DiNiu_ws")),
                 "src/diuniu_nav/maps/map.yaml"))

class P(Node):
    def __init__(self):
        super().__init__("pose_map_dist")
        self.tf = Buffer()
        self.lis = TransformListener(self.tf, self)

rclpy.init()
n = P()
t = None
for _ in range(30):  # AMCL 未定位前 map 帧不存在，轮询等待
    rclpy.spin_once(n, timeout_sec=0.3)
    try:
        t = n.tf.lookup_transform("map", "base_link", rclpy.time.Time())
        break
    except Exception:
        continue
if t is None:
    print("TF map->base_link 等待 9s 仍不可用（AMCL 未定位？）")
    rclpy.shutdown()
    raise SystemExit(1)

stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
now = n.get_clock().now().nanoseconds * 1e-9
x = t.transform.translation.x
y = t.transform.translation.y
qz = t.transform.rotation.z
qw = t.transform.rotation.w
yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
print(f"AMCL 位姿: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°  TF滞后={now-stamp:.1f}s")

with open(MAP_YAML) as f:
    m = yaml.safe_load(f)
img_path = MAP_YAML.rsplit("/", 1)[0] + "/" + m["image"]
# 读 pgm (P5)
with open(img_path, "rb") as f:
    magic = f.readline().strip()
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split())
    maxval = int(f.readline())
    data = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)
res = m["resolution"]
ox, oy = m["origin"][0], m["origin"][1]
occ = data < 128  # 占用(黑)
ys, xs = np.nonzero(occ)
wx = ox + (xs + 0.5) * res
wy = oy + (H - 1 - ys + 0.5) * res

dx = wx - x
dy = wy - y
d = np.hypot(dx, dy)
print(f"全场最近占用格: {d.min():.2f}m")

# 正前方 ±45° 内最近
ang = np.arctan2(dy, dx) - yaw
ang = (ang + math.pi) % (2 * math.pi) - math.pi
front = np.abs(ang) < math.radians(45)
rear = np.abs(ang) > math.radians(135)
if front.any():
    print(f"正前±45° 最近占用格: {d[front].min():.2f}m  方向内占用格数={front.sum()}")
if rear.any():
    print(f"正后±45° 最近占用格: {d[rear].min():.2f}m")
else:
    print("正后±45° 无占用格")
rclpy.shutdown()
