import rclpy, time
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

rclpy.init()
node = rclpy.create_node("cloud_z_probe")
holder = {"msg": None}
node.create_subscription(PointCloud2, "/cloud_registered_body", lambda m: holder.__setitem__("msg", m), 10)
t0 = time.time()
while holder["msg"] is None and time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.2)
msg = holder["msg"]
assert msg is not None, "没收到点云"
print("frame:", msg.header.frame_id)

pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
arr = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
print("点数:", len(arr))
z = arr[:, 2]
print("z 分布: min=%.2f  p5=%.2f  p25=%.2f  中位=%.2f  p75=%.2f  p95=%.2f  max=%.2f" % (
    z.min(), np.percentile(z, 5), np.percentile(z, 25), np.median(z),
    np.percentile(z, 75), np.percentile(z, 95), z.max()))
r = np.linalg.norm(arr[:, :2], axis=1)
for lo, hi in [(0.10, 1.20), (-1.70, -1.40), (-0.10, 0.10)]:
    m = (z >= lo) & (z <= hi)
    print("z∈[%+.2f,%+.2f]: %d 点, 水平距离 %.1f~%.1f m" % (lo, hi, m.sum(), r[m].min() if m.any() else -1, r[m].max() if m.any() else -1))
node.destroy_node(); rclpy.shutdown()
