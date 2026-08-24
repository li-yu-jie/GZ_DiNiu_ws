import rclpy, math, time
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from PIL import Image

MAP_PGM = "/home/y/GZ_DiNiu_ws/src/diuniu_nav/maps/map.pgm"
ORIGIN = (-15.8, -52.4)
RES = 0.05

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
tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

arr = np.asarray(point_cloud2.read_points_numpy(cloud, field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
img = np.array(Image.open(MAP_PGM))
H, W = img.shape
occ = img < 100

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

band_match(-1.40, 0.0,  "新带 z[-1.40, 0.0] (地面0.2~1.6m)")
band_match( 0.10, 1.20, "旧带 z[+0.10,+1.20] (地面1.7~2.8m)")
band_match( 1.20, 2.40, "头顶 z[+1.20,+2.40] (地面2.8~4.0m)")
band_match(-1.65,-1.40, "地面带 z[-1.65,-1.40] (地面0~0.2m)")
node.destroy_node(); rclpy.shutdown()
