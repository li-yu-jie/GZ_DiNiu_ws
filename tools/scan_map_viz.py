import rclpy, math, time
import numpy as np
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PIL import Image

MAP_PGM = "/home/y/GZ_DiNiu_ws/src/diuniu_nav/maps/map.pgm"
ORIGIN = (-15.8, -52.4)
RES = 0.05

rclpy.init()
node = rclpy.create_node("scan_map_viz")
tf_buf = Buffer()
TransformListener(tf_buf, node)

scan_holder = {"msg": None}
qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, "/scan_filtered", lambda m: scan_holder.__setitem__("msg", m), qos)

t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
    if scan_holder["msg"] is not None and time.time() - t0 > 3:
        break
scan = scan_holder["msg"]
assert scan is not None, "没收到 /scan_filtered"

tf = tf_buf.lookup_transform("map", scan.header.frame_id, rclpy.time.Time(),
                             timeout=rclpy.duration.Duration(seconds=2.0))
tx, ty = tf.transform.translation.x, tf.transform.translation.y
q = tf.transform.rotation
yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

img = np.array(Image.open(MAP_PGM).convert("RGB"))
H, W, _ = img.shape

def to_px(x, y):
    cx = int((x - ORIGIN[0]) / RES)
    row = H - 1 - int((y - ORIGIN[1]) / RES)
    return cx, row

# 画扫描点(红)
ang = scan.angle_min
n = 0
for r in scan.ranges:
    if math.isfinite(r) and scan.range_min < r < scan.range_max:
        px = tx + r * math.cos(yaw + ang)
        py = ty + r * math.sin(yaw + ang)
        cx, row = to_px(px, py)
        if 0 <= row < H and 0 <= cx < W:
            img[max(0,row-1):row+2, max(0,cx-1):cx+2] = [255, 0, 0]
            n += 1
    ang += scan.angle_increment

# 画机器人位置(蓝十字)
cx, row = to_px(tx, ty)
if 0 <= row < H and 0 <= cx < W:
    img[max(0,row-8):row+8, max(0,cx-2):cx+2] = [0, 0, 255]
    img[max(0,row-2):row+2, max(0,cx-8):cx+8] = [0, 0, 255]
    # 朝向线
    for d in range(0, 20):
        hx, hr = to_px(tx + d*RES*math.cos(yaw), ty + d*RES*math.sin(yaw))
        if 0 <= hr < H and 0 <= hx < W:
            img[hr, hx] = [0, 255, 0]

Image.fromarray(img).save("/tmp/scan_on_map.png")
print("画了 %d 个扫描点, 输出 /tmp/scan_on_map.png" % n)
node.destroy_node(); rclpy.shutdown()
