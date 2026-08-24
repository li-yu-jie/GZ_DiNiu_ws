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
tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

arr = np.asarray(point_cloud2.read_points_numpy(cloud, field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
img = np.array(Image.open(MAP_PGM).convert("RGB"))
H, W, _ = img.shape
cy_, sy_ = math.cos(yaw), math.sin(yaw)

def to_px(mx, my):
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
    cx, row = to_px(mx, my)
    ok = (row >= 0) & (row < H) & (cx >= 0) & (cx < W)
    img[row[ok], cx[ok]] = color
    return ok.sum()

n1 = draw_band(-1.40, 0.0,  [255, 0, 0])    # 新低带 = 红
n2 = draw_band( 0.10, 1.20, [0, 200, 255])  # 旧高带 = 青
print("红点(低带)=%d 青点(高带)=%d" % (n1, n2))

cx, row = to_px(np.array([tx]), np.array([ty]))
img[max(0,row[0]-8):row[0]+8, max(0,cx[0]-2):cx[0]+2] = [0,0,255]
img[max(0,row[0]-2):row[0]+2, max(0,cx[0]-8):cx[0]+8] = [0,0,255]
for d in range(0, 24):
    hx, hr = to_px(np.array([tx + d*RES*cy_]), np.array([ty + d*RES*sy_]))
    img[hr[0], hx[0]] = [0, 255, 0]

Image.fromarray(img).save("/tmp/dual_band_on_map.png")
print("输出 /tmp/dual_band_on_map.png")
node.destroy_node(); rclpy.shutdown()
