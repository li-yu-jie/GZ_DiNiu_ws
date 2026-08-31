import rclpy, math, time, numpy as np
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg

class P(Node):
    def __init__(self):
        super().__init__("livox_raw_probe")
        self.sub = self.create_subscription(CustomMsg, "/livox/lidar", self.cb, 10)
        self.done = False
    def cb(self, msg):
        n = len(msg.points)
        if n == 0:
            print("raw 云为空!")
            self.done = True
            return
        xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
        r = np.linalg.norm(xyz, axis=1)
        rh = np.hypot(xyz[:, 0], xyz[:, 1])
        print(f"frame={msg.header.frame_id} 原始点数={n}")
        print(f"3D距离: 最近={r.min():.3f}m  1%分位={np.percentile(r,1):.3f}m")
        for lo, hi in [(0, 0.3), (0.3, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 50)]:
            c = ((r >= lo) & (r < hi)).sum()
            print(f"  r[{lo},{hi}): {c} 点")
        # 车头方向 (lidar 系 +x 近似车头) 2m 内点数
        front = (xyz[:, 0] > 0) & (np.abs(xyz[:, 1]) < 1.0) & (rh < 2.0)
        print(f"前方±1m宽、2m内: {front.sum()} 点")
        self.done = True

rclpy.init()
n = P()
t0 = time.time()
while rclpy.ok() and not n.done and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.5)
if not n.done:
    print("8s 内没收到 /livox/lidar")
rclpy.shutdown()
