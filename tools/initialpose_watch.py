import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Parameter

DUR = 60

class P(Node):
    def __init__(self):
        super().__init__("pose_watch")
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self.on_init, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_amcl, 10)
    def on_init(self, m):
        p = m.pose.pose
        yaw = math.degrees(math.atan2(2 * p.orientation.w * p.orientation.z,
                                      1 - 2 * p.orientation.z ** 2))
        print(f"★ /initialpose 收到: ({p.position.x:.2f},{p.position.y:.2f},{yaw:.1f}°)  stamp={m.header.stamp.sec}", flush=True)
    def on_amcl(self, m):
        p = m.pose.pose
        yaw = math.degrees(math.atan2(2 * p.orientation.w * p.orientation.z,
                                      1 - 2 * p.orientation.z ** 2))
        c = m.pose.covariance
        print(f"  /amcl_pose: ({p.position.x:.2f},{p.position.y:.2f},{yaw:.1f}°) cov_xx={c[0]:.2f}", flush=True)

rclpy.init()
n = P()

# 顺手读 AMCL 关键参数
cli = n.create_client(GetParameters, "/amcl/get_parameters")
if cli.wait_for_service(timeout_sec=8):
    req = GetParameters.Request()
    req.names = ["update_min_d", "update_min_a", "set_initial_pose",
                 "initial_pose.x", "initial_pose.y", "initial_pose.yaw",
                 "transform_tolerance", "laser_max_range", "max_beams"]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=8)
    if fut.result():
        for name, val in zip(req.names, fut.result().values):
            print(f"参数 {name} = {val.double_value if val.type==3 else (val.bool_value if val.type==1 else val.integer_value)}")
else:
    print("amcl 参数服务不可用")

print(f"\n监听 /initialpose 与 /amcl_pose {DUR}s ...")
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.5)
print("监听结束")
rclpy.shutdown()
