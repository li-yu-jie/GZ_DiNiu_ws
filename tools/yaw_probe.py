import rclpy, math, time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def yaw_of(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

rclpy.init()
node = rclpy.create_node("yaw_probe")
data = {"imu": None, "flio": None, "ekf": None}
node.create_subscription(Imu, "/imu/data", lambda m: data.__setitem__("imu", yaw_of(m.orientation)), 10)
node.create_subscription(Odometry, "/odom", lambda m: data.__setitem__("flio", yaw_of(m.pose.pose.orientation)), 10)
node.create_subscription(Odometry, "/odometry/filtered", lambda m: data.__setitem__("ekf", yaw_of(m.pose.pose.orientation)), 10)

print("  t | BNO085 yaw | FAST-LIO yaw | EKF yaw")
for i in range(14):
    rclpy.spin_once(node, timeout_sec=0.3)
    time.sleep(0.2)
    def f(v): return ("%8.2f" % v) if v is not None else "     ---"
    print("  %02d | %s | %s | %s" % (i, f(data["imu"]), f(data["flio"]), f(data["ekf"])))
node.destroy_node(); rclpy.shutdown()
