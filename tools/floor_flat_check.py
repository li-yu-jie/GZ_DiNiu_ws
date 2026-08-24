import rclpy, time
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

for name in ("raw", "lvl"):
    arr = np.asarray(point_cloud2.read_points_numpy(got[name], field_names=("x","y","z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
    r = np.linalg.norm(arr[:, :2], axis=1)
    print("=== %s ===" % name)
    # 地面候选点: z < -1.2 (两朵云地面都应在 -1.6 附近)
    for lo, hi in [(10, 13), (13, 16), (16, 20), (20, 26)]:
        m = (r >= lo) & (r < hi) & (arr[:, 2] < -1.2)
        if m.sum() > 3:
            print("  %2d~%2dm: %4d 个低点, z 中位=%6.3f  z min=%6.3f" % (lo, hi, m.sum(), np.median(arr[m, 2]), arr[m, 2].min()))
        else:
            print("  %2d~%2dm: %4d 个低点" % (lo, hi, m.sum()))
node.destroy_node(); rclpy.shutdown()
