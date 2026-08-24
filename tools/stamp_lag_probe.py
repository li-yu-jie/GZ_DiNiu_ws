import rclpy, time
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node("stamp_lag_probe")

def mk_cb(name):
    def cb(m):
        now = node.get_clock().now().nanoseconds * 1e-9
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        print("%-24s 滞后=%8.3fs  stamp=%.3f  now=%.3f" % (name, now - st, st, now))
    return cb

node.create_subscription(PointCloud2, "/cloud_registered_body", mk_cb("cloud_registered_body"), 10)
node.create_subscription(PointCloud2, "/cloud_leveled", mk_cb("cloud_leveled"), 10)
node.create_subscription(Odometry, "/odom", mk_cb("odom(FAST-LIO)"), 10)
qos = QoSProfile(depth=10); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, "/scan", mk_cb("scan"), qos)

t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node(); rclpy.shutdown()
