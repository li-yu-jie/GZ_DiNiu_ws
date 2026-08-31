import rclpy, math, time, sys
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from PIL import Image
from common import load_map, quat_yaw

# 地图 origin/resolution 从 map.yaml 现读（2026-08-27 重录后 origin=[-29.4,-53,0]），勿再硬编码
img, ORIGIN, RES = load_map(mode="RGB")
H, W, _ = img.shape

rclpy.init()
node = rclpy.create_node("dual_band_viz")
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

def to_px_arr(mx, my):
    cx = ((mx - ORIGIN[0]) / RES).astype(int)
    row = H - 1 - ((my - ORIGIN[1]) / RES).astype(int)
    return cx, row

def draw_band(zlo, zhi, color):
    m = (arr[:,2] >= zlo) & (arr[:,2] <= zhi)
    pts = arr[m]
    r = np.linalg.norm(pts[:,:2], axis=1)
    pts = pts[r < 25.0]  # 25m 以内
    mx = tx + pts[:,0]*cy_ - pts[:,1]*sy_
    my = ty + pts[:,0]*sy_ + pts[:,1]*cy_
    cx, row = to_px_arr(mx, my)
    ok = (row >= 0) & (row < H) & (cx >= 0) & (cx < W)
    img[row[ok], cx[ok]] = color
    return ok.sum()

# 切片带 z∈[0.20, 1.20]：2026-08-28 起 /cloud_leveled 为真实 base_link（z=0 在地面），
# z 即地面高度。（此前 z 原点在雷达：8-27 雷达移 0.66m 正装后切片带为 z∈[-0.46,0.54]，
# 更早 1.6m 桅杆时代为 z∈[-1.40,0.0]，均已过期）
n1 = draw_band(0.20, 1.20, [255, 0, 0])     # 切片带(低) = 红
n2 = draw_band(1.20, 1.90, [0, 200, 255])   # 上带(高) = 青
print("红点(低带)=%d 青点(高带)=%d" % (n1, n2))

cx, row = to_px_arr(np.array([tx]), np.array([ty]))
img[max(0,row[0]-8):row[0]+8, max(0,cx[0]-2):cx[0]+2] = [0,0,255]
img[max(0,row[0]-2):row[0]+2, max(0,cx[0]-8):cx[0]+8] = [0,0,255]
for d in range(0, 24):
    hx, hr = to_px_arr(np.array([tx + d*RES*cy_]), np.array([ty + d*RES*sy_]))
    img[hr[0], hx[0]] = [0, 255, 0]

Image.fromarray(img).save("/tmp/dual_band_on_map.png")
print("输出 /tmp/dual_band_on_map.png")
node.destroy_node(); rclpy.shutdown()
