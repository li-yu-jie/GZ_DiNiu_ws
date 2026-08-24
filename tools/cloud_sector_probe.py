import rclpy, time
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

rclpy.init()
node = rclpy.create_node("cloud_sector_probe")
holder = {"msg": None}
node.create_subscription(PointCloud2, "/cloud_registered_body", lambda m: holder.__setitem__("msg", m), 10)
t0 = time.time()
while holder["msg"] is None and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
msg = holder["msg"]
arr = np.asarray(point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True), dtype=np.float32).reshape(-1, 3)
r = np.linalg.norm(arr[:, :2], axis=1)

# 最近的点簇(墙): 距离最近的 200 个点
idx = np.argsort(r)[:200]
near = arr[idx]
print("最近 200 点: 距离 %.2f~%.2f m" % (r[idx].min(), r[idx].max()))
print("  其 z 范围: %.2f ~ %.2f" % (near[:, 2].min(), near[:, 2].max()))
zs = np.sort(near[:, 2])
print("  z 十分位:", np.round(np.percentile(zs, [0,10,25,50,75,90,100]), 2))

# 所有点按距离环带统计 z 最小值(地面包络)
for lo, hi in [(1.5, 3), (3, 5), (5, 8), (8, 12), (12, 20)]:
    m = (r >= lo) & (r < hi)
    if m.sum() > 0:
        print("距离 %4.1f~%4.1fm: %4d 点, z min=%6.2f  z p10=%6.2f  z max=%6.2f" % (
            lo, hi, m.sum(), arr[m, 2].min(), np.percentile(arr[m, 2], 10), arr[m, 2].max()))
node.destroy_node(); rclpy.shutdown()
