import rclpy, math, time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("amcl_stability")
tf_buf = Buffer()
TransformListener(tf_buf, node)
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)
print("AMCL 位姿 20 秒采样（静止车辆应几乎不动）:")
t0 = time.time()
while time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tf = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        q = tf.transform.rotation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        print("  %4.1fs: x=%7.3f y=%7.3f yaw=%8.2f°" % (
            time.time()-t0, tf.transform.translation.x, tf.transform.translation.y, yaw))
    except Exception as e:
        print("  TF 失败: %s" % e)
    time.sleep(2.0)
node.destroy_node(); rclpy.shutdown()
