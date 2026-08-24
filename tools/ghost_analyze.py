import rclpy, math, time
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PIL import Image

MAP_PGM = "/home/y/GZ_DiNiu_ws/src/diuniu_nav/maps/map.pgm"
ORIGIN = (-15.8, -52.4)
RES = 0.05

rclpy.init()
node = rclpy.create_node("ghost_analyze")
tf_buf = Buffer()
TransformListener(tf_buf, node)
cloud_h = {"m": None}
scan_h = {"m": None}
node.create_subscription(PointCloud2, "/cloud_leveled", lambda m: cloud_h.__setitem__("m", m), 10)
qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, "/scan", lambda m: scan_h.__setitem__("m", m), qos)
t0 = time.time()
while (cloud_h["m"] is None or scan_h["m"] is None) and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
t1 = time.time()
while time.time() - t1 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

cloud, scan = cloud_h["m"], scan_h["m"]
tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

arr = np.asarray(point_cloud2.read_points_numpy(cloud, field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
r = np.linalg.norm(arr[:, :2], axis=1)

# ---- 1) 带内(r>3m) z 直方图 ----
band = (arr[:,2] >= -1.40) & (arr[:,2] <= 0.0) & (r > 3.0)
zb = arr[band, 2]
print("\n带内 z∈[-1.40,0] 且 r>3m: %d 点" % len(zb))
edges = np.arange(-1.45, 0.05, 0.10)
hist, _ = np.histogram(zb, bins=edges)
for i, h in enumerate(hist):
    bar = "#" * min(h // 3, 60)
    print("  z[%+.2f,%+.2f): %4d %s" % (edges[i], edges[i+1], h, bar))

# ---- 2) 带外对照 ----
for lo, hi, tag in [(-1.70, -1.40, "带下沿外(地面0~0.2m)"), (0.0, 0.30, "带上沿外(地面1.6~1.9m)")]:
    m2 = (arr[:,2] >= lo) & (arr[:,2] < hi) & (r > 3.0)
    print("对照 %s: %d 点" % (tag, m2.sum()))

# ---- 3) /scan 回波(r>3m)对应的云点 z ----
ang = scan.angle_min
scan_pts = []
for rr in scan.ranges:
    if math.isfinite(rr) and 3.0 < rr < 30.0:
        scan_pts.append((ang, rr))
    ang += scan.angle_increment
print("\n/scan 中 r>3m 的回波数: %d" % len(scan_pts))
z_corr = []
for a, rr in scan_pts:
    da = np.abs(np.arctan2(np.sin(arr[:,1] - 0), arr[:,0]) - a)  # 粗方位
    bearing = np.arctan2(arr[:,1], arr[:,0])
    da = np.abs(np.angle(np.exp(1j*(bearing - a))))
    m3 = (da < math.radians(1.0)) & (np.abs(r - rr) < 0.4)
    if m3.sum() > 0:
        z_corr.append(arr[m3, 2].mean())
z_corr = np.array(z_corr) if z_corr else np.array([np.nan])
print("对应云点 z: 中位=%.2f  min=%.2f  max=%.2f" % (np.nanmedian(z_corr), np.nanmin(z_corr), np.nanmax(z_corr)))
hist2, _ = np.histogram(z_corr, bins=edges)
for i, h in enumerate(hist2):
    if h: print("  z[%+.2f,%+.2f): %4d" % (edges[i], edges[i+1], h))

# ---- 4) 带内点按 z 着色撒到地图上 ----
img = np.array(Image.open(MAP_PGM).convert("RGB"))
H, W, _ = img.shape
cy_, sy_ = math.cos(yaw), math.sin(yaw)
def scatter(zlo, zhi, color):
    m4 = (arr[:,2] >= zlo) & (arr[:,2] < zhi) & (r > 1.2) & (r < 30)
    pts = arr[m4]
    mx = tx + pts[:,0]*cy_ - pts[:,1]*sy_
    my = ty + pts[:,0]*sy_ + pts[:,1]*cy_
    cx = ((mx - ORIGIN[0]) / RES).astype(int)
    row = H - 1 - ((my - ORIGIN[1]) / RES).astype(int)
    ok = (row >= 0) & (row < H) & (cx >= 0) & (cx < W)
    img[row[ok], cx[ok]] = color
    return ok.sum()
n1 = scatter(-1.40, -1.20, [255, 0, 0])    # 带底 = 红
n2 = scatter(-1.20, -0.60, [255, 200, 0])  # 带中 = 黄
n3 = scatter(-0.60,  0.00, [0, 200, 255])  # 带顶 = 青
print("\n着色: 红=带底[-1.4,-1.2) %d点  黄=带中[-1.2,-0.6) %d点  青=带顶[-0.6,0] %d点" % (n1, n2, n3))
Image.fromarray(img).save("/tmp/ghost_z_on_map.png")
print("输出 /tmp/ghost_z_on_map.png")
node.destroy_node(); rclpy.shutdown()
