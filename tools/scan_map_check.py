import rclpy, math, time
from tf2_ros import Buffer, TransformListener
from common import load_map, to_px, quat_yaw, scan_xy, subscribe_scan

# 地图 origin/resolution 从 map.yaml 现读（2026-08-27 重录后 origin=[-29.4,-53,0]），勿再硬编码
img, ORIGIN, RES = load_map()
H, W = img.shape
occ = img < 100   # 黑色=占用

rclpy.init()
node = rclpy.create_node("scan_map_check")
tf_buf = Buffer()
TransformListener(tf_buf, node)

scan_holder = {"msg": None}
subscribe_scan(node, "/scan_filtered", lambda m: scan_holder.__setitem__("msg", m))

t0 = time.time()
while scan_holder["msg"] is None and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
scan = scan_holder["msg"]
if scan is None:
    print("没收到 /scan_filtered"); raise SystemExit

time.sleep(0.5)
t1 = time.time()
while time.time() - t1 < 3.0:          # 持续 spin 让 TF buffer 填起来
    rclpy.spin_once(node, timeout_sec=0.1)
try:
    tf = tf_buf.lookup_transform("map", scan.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
except Exception as e:
    print("拿不到 map->%s TF: %s" % (scan.header.frame_id, e)); raise SystemExit

tx, ty = tf.transform.translation.x, tf.transform.translation.y
yaw = quat_yaw(tf.transform.rotation)
print("机器人在 map 系位姿: x=%.2f y=%.2f yaw=%.1f°" % (tx, ty, math.degrees(yaw)))

def dilation(m, r):
    from scipy.ndimage import binary_dilation
    return binary_dilation(m, iterations=r)
try:
    occ_d = dilation(occ, 2)   # 2px = 10cm 容差
except Exception:
    occ_d = occ

cy_, sy_ = math.cos(yaw), math.sin(yaw)
n_hit, n_tot, pts_free_far = 0, 0, []
for x, y in scan_xy(scan):
    r = math.hypot(x, y)
    if scan.range_min < r < scan.range_max:
        px = tx + x*cy_ - y*sy_
        py = ty + x*sy_ + y*cy_
        # 图像行号含 H-1 翻转: ROS map_server 的 pgm 原点在左下角, 图像存储从上往下
        cx, row = to_px(px, py, ORIGIN, RES, H)
        if 0 <= row < H and 0 <= cx < W:
            n_tot += 1
            if occ_d[row, cx]:
                n_hit += 1
            elif r < 20.0:
                # 记录"实际测到东西但地图是空的"的点, 取前 5 个
                if len(pts_free_far) < 8 and not occ[row, cx]:
                    pts_free_far.append((round(px,2), round(py,2), round(r,2)))

print("有效点: %d, 落在占用格(±10cm): %d (%.1f%%)" % (n_tot, n_hit, 100.0*n_hit/max(n_tot,1)))
print("扫描有回波但地图为空 的示例点(map系 x,y,距离):", pts_free_far)
node.destroy_node(); rclpy.shutdown()
