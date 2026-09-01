# common.py — tools/ 诊断脚本公共模块
#
# 用途：集中存放各探针重复实现的公共逻辑，避免各自硬编码过期常量：
#   - 地图元数据（map.yaml 的 origin/resolution/pgm 路径）一律从
#     src/diuniu_nav/maps/map.yaml 现读（2026-08-27 重录后 origin 为
#     [-29.4, -53, 0]，旧脚本烤死的 (-15.8, -52.4) 整体偏 13.6m，勿再硬编码）。
#   - 四元数→yaw 用纯 math 实现（rootfs 无 tf_transformations）。
#   - LaserScan→(x,y) 迭代与 BEST_EFFORT QoS 订阅统一封装。
#
# 坐标系约定：2026-08-28 起 /cloud_leveled 为真实 base_link（z=0 在地面，
# 切片带 z∈[0.20,1.20]m）；此前 z 原点在雷达（旧数据切片带 z∈[-0.46,0.54]，
# 更早 1.6m 桅杆时代为 z∈[-1.40,0.0]）。/cloud_registered_body（原始 FAST-LIO）
# 不受影响，仍是雷达系。
#
# 本模块不在顶层 import rclpy / numpy / PIL（按需在用到的函数内 import），
# 保证任何脚本都能直接 from common import ... 而不受导入顺序影响。
import math
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_YAML = os.path.join(REPO_ROOT, "src", "diuniu_nav", "maps", "map.yaml")


def load_map_meta(yaml_path=MAP_YAML):
    """解析 map.yaml，返回 (pgm_path, origin, resolution)。

    pgm_path 为绝对路径（相对 map.yaml 所在目录解析）；origin 为 (x, y) 二元组。
    优先用 pyyaml（rootfs 里有），没有则退化为按行解析。
    """
    image = None
    origin = None
    res = None
    try:
        import yaml
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = meta["image"]
        origin = meta["origin"]
        res = float(meta["resolution"])
    except ImportError:
        with open(yaml_path) as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if line.startswith("image:"):
                    image = line.split(":", 1)[1].strip()
                elif line.startswith("resolution:"):
                    res = float(line.split(":", 1)[1].strip())
                elif line.startswith("origin:"):
                    raw = line.split(":", 1)[1].strip().strip("[]")
                    origin = [float(v) for v in raw.split(",")]
    if image is None or origin is None or res is None:
        raise RuntimeError("map.yaml 解析失败: %s" % yaml_path)
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    return image, (float(origin[0]), float(origin[1])), res


def load_map(yaml_path=MAP_YAML, mode=None):
    """读地图 PGM（沿用现有工具的 PIL），返回 (img_array, origin, resolution)。

    mode="RGB" 时先 convert("RGB")（需要撒点着色的工具用）；默认按原灰度读。
    """
    import numpy as np
    from PIL import Image
    pgm, origin, res = load_map_meta(yaml_path)
    im = Image.open(pgm)
    if mode is not None:
        im = im.convert(mode)
    return np.array(im), origin, res


def to_px(x, y, origin, res, height):
    """世界坐标(map 系) → 图像像素 (col, row)。

    ROS map_server 的 pgm 原点在左下角，而图像行从上往下存储，
    因此行号必须做 H-1 翻转（漏翻转会把投影上下画反）。
    """
    import math
    # 用 floor 而非 int()：int() 向零截断，负世界坐标会差一格像素
    # （map_server 的栅格划分等价于 floor）
    col = math.floor((x - origin[0]) / res)
    row = height - 1 - math.floor((y - origin[1]) / res)
    return col, row


def quat_yaw(q, y=None, z=None, w=None):
    """四元数 → yaw(rad)。

    两种用法：quat_yaw(q) 传 geometry_msgs Quaternion 对象，
    或 quat_yaw(x, y, z, w) 传四个分量。纯 math 实现，无需 tf_transformations。
    """
    if y is None:
        x_, y_, z_, w_ = q.x, q.y, q.z, q.w
    else:
        x_, y_, z_, w_ = q, y, z, w
    return math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))


def scan_xy(msg):
    """LaserScan → (x, y) 迭代器（传感器/激光系坐标），跳过 inf/nan 点。

    只滤非有限值；range_min/range_max 等进一步过滤由调用方按各自需求处理
    （各探针原本的过滤条件不完全一致）。angle 随所有 beams 递增，与原逐束
    累加 angle_increment 的写法等价。
    """
    ang = msg.angle_min
    for r in msg.ranges:
        if math.isfinite(r):
            yield r * math.cos(ang), r * math.sin(ang)
        ang += msg.angle_increment


def subscribe_scan(node, topic, callback):
    """以 BEST_EFFORT QoS 订阅 LaserScan（雷达驱动/合成 scan 多为 sensor_data QoS，
    用默认 RELIABLE 会收不到）。返回 subscription 对象。"""
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return node.create_subscription(LaserScan, topic, callback, qos)
