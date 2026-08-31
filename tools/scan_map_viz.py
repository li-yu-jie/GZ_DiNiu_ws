import rclpy, math, time
from tf2_ros import Buffer, TransformListener
from PIL import Image
from common import load_map, to_px, quat_yaw, scan_xy, subscribe_scan

# 地图 origin/resolution 从 map.yaml 现读（2026-08-27 重录后 origin=[-29.4,-53,0]），勿再硬编码
img, ORIGIN, RES = load_map(mode="RGB")
H, W, _ = img.shape

rclpy.init()
node = rclpy.create_node("scan_map_viz")
tf_buf = Buffer()
TransformListener(tf_buf, node)

scan_holder = {"msg": None}
subscribe_scan(node, "/scan_filtered", lambda m: scan_holder.__setitem__("msg", m))

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
yaw = quat_yaw(tf.transform.rotation)
print("机器人: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

# 画扫描点(红)
cy_, sy_ = math.cos(yaw), math.sin(yaw)
n = 0
for x, y in scan_xy(scan):
    r = math.hypot(x, y)
    if scan.range_min < r < scan.range_max:
        px = tx + x*cy_ - y*sy_
        py = ty + x*sy_ + y*cy_
        cx, row = to_px(px, py, ORIGIN, RES, H)
        if 0 <= row < H and 0 <= cx < W:
            img[max(0,row-1):row+2, max(0,cx-1):cx+2] = [255, 0, 0]
            n += 1

# 画机器人位置(蓝十字)
cx, row = to_px(tx, ty, ORIGIN, RES, H)
if 0 <= row < H and 0 <= cx < W:
    img[max(0,row-8):row+8, max(0,cx-2):cx+2] = [0, 0, 255]
    img[max(0,row-2):row+2, max(0,cx-8):cx+8] = [0, 0, 255]
    # 朝向线
    for d in range(0, 20):
        hx, hr = to_px(tx + d*RES*cy_, ty + d*RES*sy_, ORIGIN, RES, H)
        if 0 <= hr < H and 0 <= hx < W:
            img[hr, hx] = [0, 255, 0]

Image.fromarray(img).save("/tmp/scan_on_map.png")
print("画了 %d 个扫描点, 输出 /tmp/scan_on_map.png" % n)
node.destroy_node(); rclpy.shutdown()
