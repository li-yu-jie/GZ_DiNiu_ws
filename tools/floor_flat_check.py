import rclpy, time, sys
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

rclpy.init()
node = rclpy.create_node("floor_flat_check")
got = {}
def mk(name):
    def cb(m): got[name] = m
    return cb
node.create_subscription(PointCloud2, "/cloud_registered_body", mk("raw"), 10)
node.create_subscription(PointCloud2, "/cloud_leveled", mk("lvl"), 10)
t0 = time.time()
while ("raw" not in got or "lvl" not in got) and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)

TOPIC_OF = {"raw": "/cloud_registered_body", "lvl": "/cloud_leveled"}
missing = [TOPIC_OF[n] for n in ("raw", "lvl") if n not in got]
if missing:
    print("超时无数据，请检查上游节点是否在发: %s" % ", ".join(missing))
    sys.exit(1)

# 地面候选点阈值(地面以上 0.4m 以内)：两朵云 z 原点不同——
#   raw=/cloud_registered_body 为雷达系（0.66m 正装，地面 z≈-0.66）
#   lvl=/cloud_leveled 自 2026-08-28 起为真实 base_link（z=0 在地面）
FLOOR_Z_MAX = {"raw": -0.26, "lvl": 0.40}

for name in ("raw", "lvl"):
    arr = np.asarray(point_cloud2.read_points_numpy(got[name], field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
    r = np.linalg.norm(arr[:, :2], axis=1)
    print("=== %s ===" % name)
    for lo, hi in [(10, 13), (13, 16), (16, 20), (20, 26)]:
        m = (r >= lo) & (r < hi) & (arr[:, 2] < FLOOR_Z_MAX[name])
        if m.sum() > 3:
            print("  %2d~%2dm: %4d 个低点, z 中位=%6.3f  z min=%6.3f" % (lo, hi, m.sum(), np.median(arr[m, 2]), arr[m, 2].min()))
        else:
            print("  %2d~%2dm: %4d 个低点" % (lo, hi, m.sum()))
node.destroy_node(); rclpy.shutdown()
