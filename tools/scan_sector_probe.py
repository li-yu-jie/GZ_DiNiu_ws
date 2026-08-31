import rclpy, math, time, collections
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class P(Node):
    def __init__(self):
        super().__init__("scan_probe")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos)
        self.done = False
    def cb(self, msg):
        beams = msg.ranges
        amin, ainc = msg.angle_min, msg.angle_increment
        front, rear, allv = [], [], []
        q = collections.Counter()
        for i, v in enumerate(beams):
            if math.isinf(v) or math.isnan(v):
                continue
            allv.append(v)
            a = math.degrees(amin + i * ainc)
            if abs(a) < 45:
                q["front±45"] += 1; front.append(v)
            elif a > 135 or a < -135:
                q["rear±45"] += 1; rear.append(v)
            elif 45 <= a <= 135:
                q["left"] += 1
            else:
                q["right"] += 1
        print(f"frame={msg.header.frame_id} beams={len(beams)} valid={len(allv)}")
        if allv:
            print(f"全场最近={min(allv):.3f}m")
        print(f"正前±45°: {len(front)}点" + (f" 最近={min(front):.3f}m" if front else " ★无点"))
        print(f"正后±45°: {len(rear)}点" + (f" 最近={min(rear):.3f}m" if rear else " ★无点"))
        print("象限:", dict(q))
        self.done = True

rclpy.init()
n = P()
t0 = time.time()
while rclpy.ok() and not n.done and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.5)
rclpy.shutdown()
