import rclpy, math, time, sys
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from common import load_map, quat_yaw

# 地图 origin/resolution 从 map.yaml 现读（2026-08-27 重录后 origin=[-29.4,-53,0]），勿再硬编码
img, ORIGIN, RES = load_map()
H, W = img.shape
occ = img < 100

rclpy.init()
node = rclpy.create_node("dual_band_check")
tf_buf = Buffer()
TransformListener(tf_buf, node)
holder = {"msg": None}
node.create_subscription(PointCloud2, "/cloud_leveled", lambda m: holder.__setitem__("msg", m), 10)
t0 = time.time()
while holder["msg"] is None and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
t1 = time.time()
while time.time() - t1 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)
cloud = holder["msg"]
if cloud is None:
    print("超时无数据，请检查上游节点是否在发 /cloud_leveled")
    sys.exit(1)
tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
yaw = quat_yaw(tf.transform.rotation)
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

arr = np.asarray(point_cloud2.read_points_numpy(cloud, field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)

cy_, sy_ = math.cos(yaw), math.sin(yaw)
def band_match(zlo, zhi, label):
    m = (arr[:,2] >= zlo) & (arr[:,2] <= zhi)
    pts = arr[m]
    # 投到 map 系
    mx = tx + pts[:,0]*cy_ - pts[:,1]*sy_
    my = ty + pts[:,0]*sy_ + pts[:,1]*cy_
    cx = ((mx - ORIGIN[0]) / RES).astype(int)
    row = H - 1 - ((my - ORIGIN[1]) / RES).astype(int)
    ok = (row >= 2) & (row < H-2) & (cx >= 2) & (cx < W-2)
    row, cx = row[ok], cx[ok]
    hit = 0
    for r_, c_ in zip(row, cx):
        if occ[max(0,r_-2):r_+3, max(0,c_-2):c_+3].any():  # ±10cm 容差
            hit += 1
    r = np.linalg.norm(pts[:,:2], axis=1)[ok]
    print("%s: %d 点, 命中地图 %.1f%%, 距离 %.1f~%.1fm" % (label, len(row), 100.0*hit/max(len(row),1), r.min() if len(r) else -1, r.max() if len(r) else -1))
    return mx, my, ok

# 切片带 z∈[0.20, 1.20]：2026-08-28 起 /cloud_leveled 为真实 base_link（z=0 在地面），
# z 即地面高度。（此前 z 原点在雷达：8-27 雷达移 0.66m 正装后切片带为 z∈[-0.46,0.54]，
# 更早 1.6m 桅杆时代为 z∈[-1.40,0.0]，均已过期）
band_match( 0.20, 1.20,  "切片带 z[0.20, 1.20] (地面0.2~1.2m)")
band_match( 1.20, 1.90,  "上带 z[+1.20,+1.90] (地面1.2~1.9m)")
band_match( 1.90, 3.10,  "头顶 z[+1.90,+3.10] (地面1.9~3.1m)")
band_match( 0.00, 0.20,  "地面带 z[ 0.00, 0.20] (地面0~0.2m)")
node.destroy_node(); rclpy.shutdown()
