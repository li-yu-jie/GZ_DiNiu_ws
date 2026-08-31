import rclpy, math, time, sys
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from PIL import Image
from common import load_map, quat_yaw, subscribe_scan

# 地图 origin/resolution 从 map.yaml 现读（2026-08-27 重录后 origin=[-29.4,-53,0]），勿再硬编码
img, ORIGIN, RES = load_map(mode="RGB")
H, W, _ = img.shape

rclpy.init()
node = rclpy.create_node("ghost_analyze")
tf_buf = Buffer()
TransformListener(tf_buf, node)
cloud_h = {"m": None}
scan_h = {"m": None}
node.create_subscription(PointCloud2, "/cloud_leveled", lambda m: cloud_h.__setitem__("m", m), 10)
subscribe_scan(node, "/scan", lambda m: scan_h.__setitem__("m", m))
t0 = time.time()
while (cloud_h["m"] is None or scan_h["m"] is None) and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
t1 = time.time()
while time.time() - t1 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

cloud, scan = cloud_h["m"], scan_h["m"]
if cloud is None or scan is None:
    missing = [t for t, m in (("/cloud_leveled", cloud), ("/scan", scan)) if m is None]
    print("超时无数据，请检查上游节点是否在发: %s" % ", ".join(missing))
    sys.exit(1)
tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
yaw = quat_yaw(tf.transform.rotation)
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

arr = np.asarray(point_cloud2.read_points_numpy(cloud, field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
r = np.linalg.norm(arr[:, :2], axis=1)

# 切片带 z∈[0.20, 1.20]：2026-08-28 起 /cloud_leveled 为真实 base_link（z=0 在地面），
# 切片带=地面 [0.20,1.20]m 即 z∈[0.20,1.20]。（此前 z 原点在雷达：8-27 雷达移 0.66m
# 正装后为 z∈[-0.46,0.54]，更早 1.6m 桅杆时代为 z∈[-1.40,0.0]，均已过期）
Z_LO, Z_HI = 0.20, 1.20

# ---- 1) 带内(r>3m) z 直方图 ----
band = (arr[:,2] >= Z_LO) & (arr[:,2] <= Z_HI) & (r > 3.0)
zb = arr[band, 2]
print("\n带内 z∈[0.20,1.20] 且 r>3m: %d 点" % len(zb))
edges = np.arange(0.20, 1.30, 0.10)
hist, _ = np.histogram(zb, bins=edges)
for i, h in enumerate(hist):
    bar = "#" * min(h // 3, 60)
    print("  z[%+.2f,%+.2f): %4d %s" % (edges[i], edges[i+1], h, bar))

# ---- 2) 带外对照 ----
for lo, hi, tag in [(0.0, 0.20, "带下沿外(地面0~0.2m)"), (1.20, 1.50, "带上沿外(地面1.2~1.5m)")]:
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
n1 = scatter(0.20, 0.40, [255, 0, 0])      # 带底 = 红
n2 = scatter(0.40, 0.80, [255, 200, 0])    # 带中 = 黄
n3 = scatter(0.80, 1.20, [0, 200, 255])    # 带顶 = 青
print("\n着色: 红=带底[0.20,0.40) %d点  黄=带中[0.40,0.80) %d点  青=带顶[0.80,1.20] %d点" % (n1, n2, n3))
Image.fromarray(img).save("/tmp/ghost_z_on_map.png")
print("输出 /tmp/ghost_z_on_map.png")
node.destroy_node(); rclpy.shutdown()
