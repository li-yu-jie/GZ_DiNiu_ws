import rclpy, time, math
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

class P(Node):
    def __init__(self):
        super().__init__("amcl_converge")
        self.cli = self.create_client(Empty, "/request_nomotion_update")
        self.last = None
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose",
            lambda m: setattr(self, "last", m), 10)

rclpy.init()
n = P()
if not n.cli.wait_for_service(timeout_sec=10):
    print("服务不可用")
    raise SystemExit(1)

for i in range(25):
    fut = n.cli.call_async(Empty.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    # 等一帧 amcl_pose
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(n, timeout_sec=0.2)

m = n.last
if m is None:
    print("没收到 /amcl_pose")
    raise SystemExit(1)
p = m.pose.pose
yaw = math.degrees(math.atan2(2 * p.orientation.w * p.orientation.z,
                              1 - 2 * p.orientation.z ** 2))
c = m.pose.covariance
print(f"收敛后位姿: x={p.position.x:.2f} y={p.position.y:.2f} yaw={yaw:.1f}°")
print(f"协方差: xx={c[0]:.3f} yy={c[7]:.3f} yawyaw={math.degrees(math.sqrt(c[35])):.2f}°(std)")
rclpy.shutdown()
