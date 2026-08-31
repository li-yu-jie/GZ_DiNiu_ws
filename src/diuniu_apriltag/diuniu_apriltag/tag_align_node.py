#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag 倒车对中节点（tag_align）
==================================
车尾相机识别地面码 tag0，根据【码中心坐标 + 码偏航角】输出速度指令，
两段式完成倒车对码头：倒车调距离+横向对中 → 原地自转精调偏航 → 停车
锁定；叉取动作由人接管。

控制律（distance_enable=true，两段式状态机）：
    DRIVE  调距离：v = −min(reverse_speed, max(min_creep, k_v·(z−target_z)))
            倒车，w = steer_sign·k_lat·θ_lat 纯横向对中（本段屏蔽偏航项，
            避免两项冲突 Y 轴对不准）；横向偏差越大倒车越慢（v_scale 减速），
            但不低于 creep_floor；|z−target_z|≤arrive_tol → ALIGN3；
            过冲（z<target_z）→ OVERSHOOT 只停不补
    ALIGN3 调偏航：v=0 原地自转（底盘纯旋转模式：前轮打 ±90°、绕后轴
            中心自转，|w|≥0.05 才触发），w = yaw_sign·k_yaw·yaw_err 抬过
            min_pivot_w；只对 ≤stale_w_window 内的新帧发 w（不拿旧偏航
            盲转）；收敛到 |yaw_err|≤yaw_tol → DONE 停车锁定；
            被拖离 target_z+rearm_extra → 回 DRIVE 重新武装
    DONE   锁定：发零速；被拖远 >arrive_tol+rearm_extra → DRIVE 重对；
            被推近 >arrive_tol+rearm_extra → OVERSHOOT 报警
    OVERSHOOT 报警态：只停不补（绝不自动前开），等人把车拉回远处
            （err_z>arrive_tol）自动回 DRIVE
    （θ_lat=atan2(t.x−target_x, t.z) 码中心横向角；yaw=四元数ZYX绕相机
      z 轴，与 distance_watch.py 同定义；横向与偏航各走独立符号，见下）
    distance_enable=false 退回旧行为：恒速倒车 + 全律调角，不管距离。
    （历史上曾有 ALIGN1 倒车前预旋转段，已彻底删除——DRIVE 的横向对中
      本身会把偏航带进 ALIGN3 可精调的范围，预旋转只会浪费时间+放大误差）

距离控制（distance_enable=true 时走上述状态机）：
    ★ target_z 必须现场标定：人工把车停到真实到位点，读 distance_watch 的 z
      填入。码要贴在"到位时仍在相机视野内"的位置——到位前丢码会触发
      丢码停车，永远开不到目标点。
    distance_enable=false 退回旧行为：恒速 reverse_speed 倒车 + 全律调角。

符号推导与实车标定（★横向/偏航两个符号独立，互不替代）：
    ★ 2026-08-30 实车复测最终标定：steer_sign=−1（横向对中）、yaw_sign=−1
      （偏航自转），与 8-26 记录一致，以 config/tag_align.yaml 为准
      （节点内默认值已同步。8-30 曾误改 yaw_sign=+1，实测为反方向，
      当日改回 −1）。
    ★ 两者符号必须分离——2026-08-26 实测单符号版本横向正确时偏航环为
      正反馈：原地自转 w 顶死 ±max_angular 持续打转（越转越偏，实车打转
      −68° 人为接管急停）。根因：相机安装朝向/底盘接线使横向与偏航的
      反馈方向天然相反。
    若换相机/改安装后方向调反：低速倒车，车朝码收拢=横向对，越调越偏
      =steer_sign 取反；ALIGN3 原地自转越转越偏=yaw_sign 取反。
      验证横向符号前先确认 k_yaw=0 或 yaw_target 已标定，否则测到的是偏航项。
    底层 diuniu_base 的三轮车解算 alpha=atan(wL/v) 已正确处理倒车符号，
    本节点只发标准 (v, w)，不用管打角方向换算。

安全行为：
  - 丢码（新检测 stamp 超过 lost_timeout 未到达）→ 立即发零速停车，绝不盲倒。
    注意按"新 stamp 到达"判活而非 stamp 龄期：1080p MJPG 链路端到端延迟
    可达 ~1s，活动检测的 stamp 本身就滞后，按龄期判会误杀活码
  - 内容冻结判活：stamp 在推进但位姿读数 freeze_timeout 秒不变（DRIVE/
    ALIGN3 中车本该在动）→ 相机画面冻结/USB 卡死或车轮受阻打滑 → 停车
  - ALIGN3 只对 ≤stale_w_window 内的新检测发自转 w，其余周期发零速等新帧
  - 参数校验：steer_sign/yaw_sign 只接受 ±1，增益/速度/阈值拒绝非法值
    （param set 误输不会变成危险指令）
  - enabled=false → 持续发零速（暂停），可 ros2 param set /tag_align enabled 热切换
  - 底盘 0.2s 看门狗兜底：本节点崩了车也会自停
  - 手柄 /cmd_vel_joy 优先：底盘仲裁手柄活跃期忽略本节点指令，
    松开手柄 0.5s 后本节点自动恢复倒车（注意！）
  - ★ 不要与 Nav2 同时运行（两边都发 /cmd_vel 会互相打架）

调参顺序（实车）：
  1. k_yaw=0 先只开横向项，低速倒车看车是否朝码收拢——越调越偏 → steer_sign 取反
  2. 横向正常后验偏航：ALIGN3 原地自转看是否朝 yaw_target 收敛——
     越转越偏（w 顶死限幅打转）→ yaw_sign 取反
  3. 把车摆正，读 distance_watch 的偏航读数填入 yaw_target_deg，
     再逐渐加大 k_yaw
  4. 距离控制：人工停到真实到位点，读 distance_watch 的 z 填入 target_z；
     停早了调大、停晚了调小；min_creep/creep_floor 不得低于 0.05
     （底盘纯旋转阈值，低了只打角不走车假死）
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException

from diuniu_apriltag.tf_utils import quat_yaw


def wrap_pi(a):
    """环绕到 (-π, π]"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class TagAlignNode(Node):
    """按 AprilTag 中心坐标 + 偏航角的两段式倒车对中节点（距离控制）。"""

    def __init__(self):
        super().__init__('tag_align')

        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('tag_frame', 'tag0')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('reverse_speed', 0.16)   # 倒车线速度上限 (m/s)，linear.x 为负
        self.declare_parameter('max_angular', 0.4)      # 角速度限幅 (rad/s)
        self.declare_parameter('k_lat', 1.2)            # 横向角增益 (rad/s per rad)
        self.declare_parameter('k_yaw', 0.15)           # 偏航增益 (rad/s per rad)
        self.declare_parameter('yaw_target_deg', 100.0)  # 目标偏航（度），摆正实读数；2026-08-30 实车复测基线+100°
        self.declare_parameter('steer_sign', -1.0)       # 横向对中方向符号（2026-08-30 实车复测 -1），打反则取反
        self.declare_parameter('yaw_sign', -1.0)         # 偏航自转方向符号（2026-08-30 实车复测 -1；当日曾误改 +1 已推翻），打反则取反
        self.declare_parameter('lost_timeout', 1.5)     # 无新检测判丢码 (s)。检测链 CPU 争抢时
        # 帧率可掉到 1~2fps，0.5s 会误杀活码；1.5s×0.16m/s=24cm 盲倒上限可接受
        self.declare_parameter('control_rate', 20.0)    # cmd_vel 发布频率 (Hz)，底盘看门狗 0.2s 需 >5Hz
        # （注意：control_rate 只在启动时读一次，热改需重启节点）
        self.declare_parameter('enabled', True)
        # —— 距离控制（到位停车）——
        self.declare_parameter('distance_enable', True)  # 总开关；false=恒速倒不管距离
        self.declare_parameter('target_x', 0.0)          # 目标 x 实读数 (m)，用于补偿相机左右安装偏差
        self.declare_parameter('target_z', 2.40)         # 到位时 z 实读数 (m)，★必须现场标定
        self.declare_parameter('k_v', 0.4)               # 接近段速度斜率 v=k_v·误差 (1/s)
        self.declare_parameter('min_creep', 0.12)        # 末段爬行 (m/s)，克服电机启动死区；不得低于底盘 0.05 阈值
        self.declare_parameter('creep_floor', 0.06)      # 横向减速后的速度下限 (m/s)。v_scale 横向减速
        # 可低于 min_creep 以便大偏差时慢速对中，但不低于该下限；★必须 >0.05（底盘纯旋转触发阈值），
        # 否则 |v|<0.05 且 |w|≥0.05 时车进入"前轮±90°绕后轴自转"模式，倒车对中变成原地打转
        self.declare_parameter('arrive_tol', 0.03)       # 到位死区 (m)
        self.declare_parameter('rearm_extra', 0.30)      # 到位后被拖离/推近该距离自动重新武装 (m)
        # —— 两段式（调距离 → 精调偏航）——
        self.declare_parameter('yaw_tol_deg', 3.0)       # 偏航收敛容差 (度)，ALIGN3 出口判据
        # 注意：检测链路延迟 ~1s，连续自转每一步冲过 min_pivot_w×1s≈5.7°，
        # 容差 <2~3° 会在目标两侧来回振荡永不出口；要 1° 级精度需改
        # "转-停-等新帧-再判"步进模式（未实现），当前 yaml 用 3.0°
        self.declare_parameter('min_pivot_w', 0.10)      # 原地自转最小角速度 (rad/s)，
        # 底盘纯旋转模式要求 |v|<0.05 且 |w|>=0.05 才触发（前轮±90°绕后轴自转），
        # 低于阈值只打角不走车会假死，故 ALIGN3 段把小 w 抬到该值
        self.declare_parameter('stale_w_window', 0.3)    # ALIGN3 只用该时长内的新检测发自转 w (s)，
        # 其余周期发零速等新帧——偏航随自转快速变化，拿旧读数继续转=盲转
        self.declare_parameter('freeze_timeout', 3.0)    # 内容冻结判活 (s)：stamp 在推进但位姿读数
        # 不变（DRIVE/ALIGN3 中车本该在动）→ 相机冻结/USB 卡死或车轮受阻打滑 → 停车
        self.declare_parameter('settle_time', 5.0)       # 到中对正后稳定保持时间门限 (s)

        # —— 拉前重试修正参数 ——
        self.declare_parameter('max_forward_retries', 5)
        self.declare_parameter('forward_adjust_dist', 0.25)  # 前拉距离上限 (m)：
        # 实际前拉 ∝ 横向偏差（约 3×|x 误差|，下限 0.08m），封顶此值；不再每次固定走 25cm
        self.declare_parameter('lateral_align_tol', 0.03)
        self.declare_parameter('k_x', 0.5)             # 横向"米制"增益 (rad/s per m)：
        # w 增加 k_x·(x−target_x) 项——θ_lat=atan2(x,z) 远距时角度小、纠偏弱，
        # 米制项让大横向偏差（如 26cm）获得与距离无关的强纠偏，收敛更快
        # （2026-08-30 实车验证：横向 +46.4cm → +1.5cm 单次通过收敛）

        gp = self.get_parameter
        self.camera_frame = gp('camera_frame').value
        self.tag_frame = gp('tag_frame').value
        self.reverse_speed = abs(gp('reverse_speed').value)
        self.max_angular = abs(gp('max_angular').value)
        self.k_lat = gp('k_lat').value
        self.k_yaw = gp('k_yaw').value
        self.yaw_target = math.radians(gp('yaw_target_deg').value)
        self.steer_sign = gp('steer_sign').value
        self.yaw_sign = gp('yaw_sign').value
        self.lost_timeout = gp('lost_timeout').value
        self.distance_enable = gp('distance_enable').value
        self.target_x = gp('target_x').value
        self.target_z = gp('target_z').value
        self.k_v = gp('k_v').value
        self.min_creep = abs(gp('min_creep').value)
        self.creep_floor = abs(gp('creep_floor').value)
        self.arrive_tol = abs(gp('arrive_tol').value)
        self.rearm_extra = abs(gp('rearm_extra').value)
        self.yaw_tol = math.radians(abs(gp('yaw_tol_deg').value))
        self.min_pivot_w = abs(gp('min_pivot_w').value)
        self.stale_w_window = abs(gp('stale_w_window').value)
        self.freeze_timeout = abs(gp('freeze_timeout').value)
        self.state = 'drive'        # 状态机：drive → align3 → done（+overshoot 报警态）

        # —— 拉前重试修正变量 ——
        self.max_forward_retries = gp('max_forward_retries').value
        self.forward_adjust_dist = gp('forward_adjust_dist').value
        self.lateral_align_tol = gp('lateral_align_tol').value
        self.k_x = gp('k_x').value
        self.forward_retry_count = 0
        self.start_forward_z = 0.0
        self.forward_target_dist = self.forward_adjust_dist  # 本次前拉的实际目标距离（按需自适应）

        # —— 低通滤波器缓存变量 ——
        self.filtered_x = None
        self.filtered_z = None
        self.filtered_yaw = None

        # —— 异常跳变帧剔除状态（跨丢码保持：丢码期间车静止，旧参考依然有效）——
        self.last_raw_x = None
        self.last_raw_z = None
        self.jump_reject_count = 0

        # —— 到位稳定滤波变量 ——
        self.settle_time = abs(gp('settle_time').value)
        self.settling_start_time = None

        # param set 误输防护：非法值直接拒绝，不变成危险指令
        self.add_on_set_parameters_callback(self._validate_params)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(Twist, gp('cmd_vel_topic').value, 10)
        self.timer = self.create_timer(
            1.0 / gp('control_rate').value, self.control_loop)

        # 丢码判活：跟踪最近一次"新 TF 时间戳"到达的时刻。
        # 不能用 时间戳-vs-当前时刻 判活——1080p MJPG 链路端到端延迟可达 ~1s
        # （多开一份识别链时 CPU 争抢更明显），活动检测的 stamp 本身就滞后，
        # 按 stamp 龄期判活会把活码误判成丢码。检测停止 = 没有新 stamp 到达。
        self.last_stamp_key = None
        self.last_new_stamp_time = None
        # 内容冻结判活：stamp 在推进但位姿读数不变 = 相机画面冻结（USB 卡死
        # 后驱动重发同一帧）或车轮受阻打滑，同样不能继续动
        self.last_content_key = None
        self.last_content_change_time = None

        dist_info = (f'距离控制开 target_z={self.target_z:.2f}±{self.arrive_tol:.2f}m'
                     if self.distance_enable else '距离控制关(恒速倒车)')
        self.get_logger().warn(
            f'🎯 倒车对中已启动: 速度上限 v=-{self.reverse_speed:.2f} m/s, '
            f'k_lat={self.k_lat}, k_yaw={self.k_yaw}, '
            f'yaw_target={gp("yaw_target_deg").value:+.1f}°, '
            f'steer_sign={self.steer_sign:+.0f}, yaw_sign={self.yaw_sign:+.0f}, {dist_info}, '
            f'丢码>{self.lost_timeout}s/冻结>{self.freeze_timeout}s 即停车。'
            f'手柄随时接管；勿与 Nav2 同时运行！')

    def _validate_params(self, params):
        """param set 校验：符号只认 ±1，增益非负，阈值必须为正/越过底盘阈值。"""
        for p in params:
            name, v = p.name, p.value
            ok, reason = True, ''
            if name in ('steer_sign', 'yaw_sign'):
                ok = v in (1.0, -1.0)
                reason = f'{name} 只能是 ±1.0（收到 {v}）'
            elif name in ('k_lat', 'k_yaw', 'k_v', 'k_x', 'rearm_extra', 'max_forward_retries', 'settle_time'):
                ok = v >= 0.0
                reason = f'{name} 不能为负（收到 {v}）'
            elif name in ('reverse_speed', 'max_angular', 'arrive_tol',
                          'yaw_tol_deg', 'stale_w_window', 'forward_adjust_dist',
                          'lateral_align_tol'):
                ok = v > 0.0
                reason = f'{name} 必须为正（收到 {v}）'
            elif name in ('min_creep', 'creep_floor', 'min_pivot_w'):
                ok = v > 0.05
                reason = (f'{name}={v} 低于底盘纯旋转触发阈值 0.05，'
                          f'会只打角不走车/误入自转模式')
            elif name == 'lost_timeout':
                ok = v >= 0.2
                reason = f'lost_timeout={v} 太小，检测链 ~1s 延迟下会误杀活码'
            elif name == 'freeze_timeout':
                ok = v >= 1.0
                reason = f'freeze_timeout={v} 太小，正常检测噪声会误判冻结'
            elif name == 'target_z':
                ok = v > 0.05
                reason = f'target_z={v} 低于码距异常线 0.05m'
            elif name == 'control_rate':
                ok = v >= 5.0
                reason = f'control_rate={v} 低于底盘 0.2s 看门狗所需的 5Hz'
            if not ok:
                return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def _reset_filter(self):
        self.filtered_x = None
        self.filtered_z = None
        self.filtered_yaw = None

    def stop(self, reason):
        """发零速并节流打印原因。"""
        self.pub.publish(Twist())
        self._reset_filter()
        self.get_logger().warn(f'🛑 停车: {reason}', throttle_duration_sec=1.0)

    def control_loop(self):
        if not self.get_parameter('enabled').value:
            self.stop('enabled=false（已暂停，param set 可恢复）')
            return

        # 每周期热读调参参数（本地读，开销可忽略）：
        # 现场标定/调参用 ros2 param set 立即生效，不用改 yaml 重编译重启。
        # 试好的值最后写回 config/tag_align.yaml 固化（param set 重启即丢）。
        # 非法值已被 _validate_params 挡在门外，这里 abs() 只是双保险。
        gp = self.get_parameter
        new_target = gp('target_z').value
        if new_target != self.target_z:
            self.target_z = new_target
            self.state = 'drive'    # 目标变了，回 drive 重新对
            self.forward_retry_count = 0  # 重置重试计数
            self.get_logger().warn(
                f'📝 target_z 热更新为 {new_target:.2f}m，状态机回 DRIVE 重新对')
        self.max_forward_retries = gp('max_forward_retries').value
        self.forward_adjust_dist = gp('forward_adjust_dist').value
        self.lateral_align_tol = gp('lateral_align_tol').value
        self.k_x = gp('k_x').value
        self.reverse_speed = abs(gp('reverse_speed').value)
        self.max_angular = abs(gp('max_angular').value)
        self.k_lat = gp('k_lat').value
        self.k_yaw = gp('k_yaw').value
        self.yaw_target = math.radians(gp('yaw_target_deg').value)
        self.steer_sign = gp('steer_sign').value
        self.yaw_sign = gp('yaw_sign').value
        self.lost_timeout = gp('lost_timeout').value
        self.distance_enable = gp('distance_enable').value
        self.target_x = gp('target_x').value
        self.k_v = gp('k_v').value
        self.min_creep = abs(gp('min_creep').value)
        self.creep_floor = abs(gp('creep_floor').value)
        self.arrive_tol = abs(gp('arrive_tol').value)
        self.rearm_extra = abs(gp('rearm_extra').value)
        self.yaw_tol = math.radians(abs(gp('yaw_tol_deg').value))
        self.min_pivot_w = abs(gp('min_pivot_w').value)
        self.stale_w_window = abs(gp('stale_w_window').value)
        self.freeze_timeout = abs(gp('freeze_timeout').value)
        self.settle_time = abs(gp('settle_time').value)

        try:
            t = self.tf_buf.lookup_transform(
                self.camera_frame, self.tag_frame, Time())
        except TransformException:
            self.stop(f'未检测到 {self.tag_frame}')
            return

        # 丢码判活：apriltag_ros 只在检测到码时发新 TF，检测一停就没有新
        # 时间戳到达；超 lost_timeout 无新检测 → 停车，绝不拿旧位姿盲倒
        stamp = t.header.stamp
        key = (stamp.sec, stamp.nanosec)
        now = self.get_clock().now()
        if key != self.last_stamp_key:
            self.last_stamp_key = key
            self.last_new_stamp_time = now
        silent = (now - self.last_new_stamp_time).nanoseconds / 1e9
        if silent > self.lost_timeout:
            self.stop(f'码丢失 {silent:.1f}s（无新检测）')
            return

        tr = t.transform.translation
        q = t.transform.rotation
        if tr.z < 0.05:     # 码贴到镜头上了 / 异常位姿，倒车无意义
            self.stop(f'码距离异常 z={tr.z:.2f}m')
            return

        yaw = quat_yaw(q.w, q.x, q.y, q.z)

        # 异常跳变帧剔除：视野边缘/检测链卡顿恢复后的首帧常是跳变坏帧
        # （2026-08-30 实测：丢码 7.7s 后首帧 x 从 +1.5cm 跳到 +23.2cm，
        # 直接骗过状态机触发错误前调）。车是物理连续体，相邻有效帧
        # z 跳 >0.30m / x 跳 >0.20m 必为坏帧；丢码期间车静止，旧参考有效。
        # 连拒 10 帧（0.5s）后放行——防止车被外力挪动后永远锁死。
        if self.last_raw_z is not None:
            if (abs(tr.z - self.last_raw_z) > 0.30
                    or abs(tr.x - self.last_raw_x) > 0.20):
                self.jump_reject_count += 1
                if self.jump_reject_count < 10:
                    self.pub.publish(Twist())
                    self.get_logger().warn(
                        f'🛑 丢弃异常跳变帧（z {self.last_raw_z:.2f}→{tr.z:.2f}m, '
                        f'x {self.last_raw_x:+.2f}→{tr.x:+.2f}m），'
                        f'连拒 {self.jump_reject_count}/10 帧后放行',
                        throttle_duration_sec=1.0)
                    return
        self.jump_reject_count = 0
        self.last_raw_x = tr.x
        self.last_raw_z = tr.z

        # 一阶低通滤波器（EMA）：平滑输入信号，消除转向轮的高频扭摆抖动
        f_alpha = 0.25
        if self.filtered_x is None:
            self.filtered_x = tr.x
            self.filtered_z = tr.z
            self.filtered_yaw = yaw
        else:
            self.filtered_x = f_alpha * tr.x + (1.0 - f_alpha) * self.filtered_x
            self.filtered_z = f_alpha * tr.z + (1.0 - f_alpha) * self.filtered_z
            yaw_diff = wrap_pi(yaw - self.filtered_yaw)
            self.filtered_yaw = wrap_pi(self.filtered_yaw + f_alpha * yaw_diff)

        # 使用滤波后的值进行控制运算
        calc_x = self.filtered_x
        calc_z = self.filtered_z
        calc_yaw = self.filtered_yaw

        theta_lat = math.atan2(calc_x - self.target_x, calc_z)
        # 引入 1.5 度控制死区，滤除残留噪声
        if abs(math.degrees(theta_lat)) <= 1.5:
            theta_lat = 0.0
            
        yaw_err = wrap_pi(calc_yaw - self.yaw_target)
        if abs(math.degrees(yaw_err)) <= 1.5:
            yaw_err = 0.0
            
        err_z = calc_z - self.target_z
        # distance_watch 同款读数展示（cm/度），各状态日志共用
        see = (f'距离 z: {tr.z*100:6.1f}cm  左右 x: {tr.x*100:+5.1f}cm  '
               f'上下 y: {tr.y*100:+5.1f}cm  偏航: {math.degrees(yaw):+5.1f}°')

        # 内容冻结判活：stamp 在推进（检测进程活着）但位姿读数不变——
        # 相机画面冻结（USB 卡死后驱动重发同一帧）或车轮受阻打滑。
        # 只在 DRIVE/ALIGN3 判：这两态车本该在动，读数必然变化；
        # DONE/OVERSHOOT 是停车态，读数不变是正常的。
        content = (round(tr.x, 3), round(tr.y, 3), round(tr.z, 3), round(yaw, 2))
        if content != self.last_content_key:
            self.last_content_key = content
            self.last_content_change_time = now
        frozen = (now - self.last_content_change_time).nanoseconds / 1e9
        if (self.distance_enable and self.state in ('drive', 'align3')
                and frozen > self.freeze_timeout):
            self.stop(f'位姿读数 {frozen:.1f}s 未变（相机画面冻结/USB 卡死，'
                      f'或车轮受阻打滑），按丢码停车')
            return

        cmd = Twist()

        if not self.distance_enable:
            # 旧行为：恒速倒车 + 全律调角，不管距离
            w = (self.steer_sign * self.k_lat * theta_lat
                 + self.yaw_sign * self.k_yaw * yaw_err)
            w = max(-self.max_angular, min(self.max_angular, w))
            cmd.linear.x = -self.reverse_speed
            cmd.angular.z = w
            self.pub.publish(cmd)
            self.get_logger().info(
                f'[恒速] {see} | 横向角 {math.degrees(theta_lat):+5.1f}° '
                f'偏航误差 {math.degrees(yaw_err):+5.1f}° '
                f'→ w={w:+.2f} v={cmd.linear.x:+.2f}',
                throttle_duration_sec=0.5)
            return

        # —— 两段式状态机：drive 倒车调中 → align3 原地自转精调偏航 → done ——
        if self.state == 'done':
            if err_z > self.arrive_tol + self.rearm_extra:
                self.state = 'drive'
                self.forward_retry_count = 0
                self.get_logger().warn('↩️ 已离开到位点，回 DRIVE 重新武装')
            elif err_z < -(self.arrive_tol + self.rearm_extra):
                # 双侧再武装：到位后又被推近（如叉车对挤/地面溜车）同样要报警，
                # 单侧再武装会在"被推近"时锁死在 done 零速、毫无察觉
                self.state = 'overshoot'
                self.get_logger().warn(
                    f'⚠️ 到位后又被推近 {-err_z:.2f}m（z={tr.z:.2f}m < '
                    f'目标 {self.target_z:.2f}m），转 OVERSHOOT 报警')
            else:
                self.pub.publish(cmd)   # 零速锁定
                self.get_logger().info(
                    f'[DONE 锁定] {see} | 偏航误差 {math.degrees(yaw_err):+5.1f}°',
                    throttle_duration_sec=2.0)
                return

        if self.state == 'overshoot':
            if err_z > self.arrive_tol:
                self.state = 'drive'   # 被拉回远端，重新对
                self.forward_retry_count = 0
                self.get_logger().warn('↩️ 过冲解除（已拉回），回 DRIVE')
            else:
                self.stop(f'过冲: 当前 z={tr.z:.2f}m 已小于目标 {self.target_z:.2f}m'
                          f'（超出 {-err_z:.2f}m），不自动前开；'
                          f'把车拉回远处自动重新对中')
                return

        if self.state == 'forward_adjust':
            # 前行调整阶段：按需前拉（∝横向偏差，封顶 forward_adjust_dist）拉开纠偏行程。
            # ★ 只直行不打舵（2026-08-30 实车证伪）：前调打舵在相机系方位角反馈下是
            # 正反馈——旋转效应主导测量，w 从 0.12 爬升顶死 0.40、打舵角 ~79° 满舵
            # 假死（车原地蹭不动），同时相机甩离码反复丢码。
            # 横向纠偏全部交给 DRIVE 段 k_lat+k_x 律（同日实车验证 46cm→1.5cm 单次收敛）。
            moved_dist = tr.z - self.start_forward_z
            if moved_dist >= self.forward_target_dist:
                self.state = 'drive'
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                self.get_logger().warn(
                    f'✅ 前行调整已移动 {moved_dist:.2f}m (>= {self.forward_target_dist:.2f}m)，重新进入倒车对中阶段...')
                return
            else:
                cmd.linear.x = 0.10  # 前行线速度 0.10 m/s
                cmd.angular.z = 0.0  # 直开（打舵实车证伪，见上）
                self.pub.publish(cmd)
                self.get_logger().info(
                    f'[FORWARD 前调] 已前行 {moved_dist:.2f}m / {self.forward_target_dist:.2f}m',
                    throttle_duration_sec=0.5)
                return

        if self.state == 'drive':
            if abs(err_z) <= self.arrive_tol:
                lat_err = abs(calc_x - self.target_x)
                if lat_err > self.lateral_align_tol and self.forward_retry_count < self.max_forward_retries:
                    self.forward_retry_count += 1
                    self.state = 'forward_adjust'
                    self.start_forward_z = tr.z
                    # 按需前拉：距离 ∝ 横向偏差（约 3 倍），下限 0.08m 保证有纠偏行程，
                    # 封顶 forward_adjust_dist（最大 25cm）——不再每次固定走 25cm
                    self.forward_target_dist = min(self.forward_adjust_dist,
                                                   max(0.08, 3.0 * lat_err))
                    self.pub.publish(Twist())  # 先停车
                    self.get_logger().warn(
                        f'⚠️ 接近目标点但横向未对齐 (左右 x: {tr.x*100:+.1f}cm，误差 {lat_err*100:.1f}cm > {self.lateral_align_tol*100:.1f}cm)！'
                        f'启动第 {self.forward_retry_count}/{self.max_forward_retries} 次前行调整，'
                        f'按需前拉 {self.forward_target_dist:.2f}m（上限 {self.forward_adjust_dist:.2f}m）...')
                    return
                else:
                    self.state = 'align3'
                    self.pub.publish(cmd)   # 先停一个周期再进精调
                    self.get_logger().warn(
                        f'🚗→🌀 距离就位（z={tr.z:.2f}m），左右对齐良好 (x={tr.x*100:+.1f}cm)，原地精调偏航')
                    return
            if err_z < 0.0:
                self.state = 'overshoot'
                self.pub.publish(cmd)   # 立即停车；报警文案统一在 overshoot 分支
                return
            v_mag = min(self.reverse_speed,
                        max(self.min_creep, self.k_v * err_z))
            # 横向偏差减速机制：左右横向偏差越大，越慢速倒车，给打舵对中留出足够反应时间和转弯位移空间。
            # 减速后下限是 creep_floor 而非 min_creep——min_creep(0.12) 兜底会把
            # 大横向偏差时的减速意图完全抵消（0.12 恒比 k_v 小段×0.2 大），
            # 变成"偏差越大越该慢却慢不下来"；creep_floor(0.06) 仍 >0.05 纯旋转阈值
            lat_err = abs(calc_x - self.target_x)
            v_scale = max(0.2, 1.0 - lat_err / 0.15)
            v_mag = max(self.creep_floor, v_mag * v_scale)

            # DRIVE 阶段屏蔽偏航纠偏项，只进行纯横向对中控制，避免两项冲突导致 Y 轴对不准。
            # 横向控制律 = 角度项 + 米制项：θ_lat=atan2(x,z) 远距时角度小纠偏弱，
            # k_x·(x−target_x) 提供与距离无关的强纠偏，大横向偏差（26cm 级）收敛显著加快
            x_err_m = calc_x - self.target_x
            w = self.steer_sign * (self.k_lat * theta_lat + self.k_x * x_err_m)
            w = max(-self.max_angular, min(self.max_angular, w))
            cmd.linear.x = -v_mag
            cmd.angular.z = w
            self.pub.publish(cmd)
            self.get_logger().info(
                f'[DRIVE 调距] {see} | 横向角 {math.degrees(theta_lat):+5.1f}° '
                f'横向误差 {x_err_m*100:+5.1f}cm '
                f'偏航误差 {math.degrees(yaw_err):+5.1f}° '
                f'目标 {self.target_z*100:.0f}cm → v={cmd.linear.x:+.3f} (scale={v_scale:.2f}) w={w:+.2f}',
                throttle_duration_sec=0.5)
            return

        if self.state == 'align3':
            if err_z > self.arrive_tol + self.rearm_extra:
                self.state = 'drive'   # 精调期间被拖走，重来
                self.forward_retry_count = 0
                self.settling_start_time = None
                self.pub.publish(cmd)  # 先停一个周期，别带着旧 w 进 DRIVE
                self.get_logger().warn('↩️ 精调中被拖离，回 DRIVE')
                return
            if silent > self.stale_w_window:
                self.pub.publish(cmd)
                self.settling_start_time = None  # 丢帧也重置稳定时间
                self.get_logger().info(
                    f'[ALIGN3 等新帧] {see} | 检测已 {silent:.1f}s 未更新，暂停自转',
                    throttle_duration_sec=1.0)
                return

            # 检查是否满足对正收敛条件（偏航误差在容差范围内）
            if abs(yaw_err) <= self.yaw_tol:
                if self.settling_start_time is None:
                    self.settling_start_time = now
                    self.get_logger().warn(
                        f'⏱️ 已达到对准范围，开始 {self.settle_time:.1f} 秒稳定度倒计时...')
                
                elapsed = (now - self.settling_start_time).nanoseconds / 1e9
                if elapsed >= self.settle_time:
                    # ★ 锁定前复查横向（2026-08-31 实车暴露）：原地自转绕后轴旋转，
                    # 精调偏航会把码的表观 x 平移 ≈ z·Δyaw（2.3m×3°≈12cm）——
                    # 进 ALIGN3 前对中的 x 会被自转本身破坏，实车曾 x=-10.2cm 锁定。
                    # 横向超差则转前调重对，不直接锁定。
                    lat_err = abs(calc_x - self.target_x)
                    if (lat_err > self.lateral_align_tol
                            and self.forward_retry_count < self.max_forward_retries):
                        self.forward_retry_count += 1
                        self.state = 'forward_adjust'
                        self.start_forward_z = tr.z
                        self.forward_target_dist = min(
                            self.forward_adjust_dist, max(0.08, 3.0 * lat_err))
                        self.settling_start_time = None
                        self.pub.publish(Twist())
                        self.get_logger().warn(
                            f'⚠️ 偏航就位但横向被自转带偏 (x={tr.x*100:+.1f}cm，'
                            f'误差 {lat_err*100:.1f}cm > {self.lateral_align_tol*100:.1f}cm)！'
                            f'第 {self.forward_retry_count}/{self.max_forward_retries} 次前调，'
                            f'前拉 {self.forward_target_dist:.2f}m 重对...')
                        return
                    self.state = 'done'
                    self.settling_start_time = None
                    self.pub.publish(cmd)
                    self.get_logger().warn(
                        f'🎉 已到位！对中圆满完成！已稳定保持 {elapsed:.1f}s，锁定！'
                        f'z={tr.z:.2f}m（目标 {self.target_z:.2f}m）x={tr.x*100:+.1f}cm '
                        f'偏航 {math.degrees(yaw_err):+.1f}°，'
                        f'拖离/推近 {self.rearm_extra:.2f}m 以上自动重新武装')
                    return
                else:
                    self.pub.publish(cmd)  # 保持静止发零速
                    self.get_logger().info(
                        f'[ALIGN3 稳定中] 偏航与距离已就位，稳定保持 {elapsed:.1f}s / {self.settle_time:.1f}s | {see}',
                        throttle_duration_sec=0.5)
                    return
            else:
                self.settling_start_time = None  # 未对准，重置稳定时间并继续微调
                w = self._pivot_w(yaw_err)
                cmd.angular.z = w
                self.pub.publish(cmd)
                self.get_logger().info(
                    f'[ALIGN3 精调偏航] {see} | '
                    f'偏航误差 {math.degrees(yaw_err):+5.1f}° '
                    f'→ 自转 w={w:+.2f} rad/s',
                    throttle_duration_sec=0.5)
                return

    def _pivot_w(self, yaw_err):
        """原地自转角速度：k_yaw 比例 + 限幅 + 抬过底盘纯旋转触发阈值。

        底盘纯旋转模式要求 |v|<0.05 且 |w|>=0.05：|w| 不足只打角不走车
        （假死），故把收敛末端的小 w 抬到 min_pivot_w；k_yaw=0 时按
        yaw_sign·yaw_err 的方向给最小自转速率（调参第一步也能走通）。
        """
        w = self.yaw_sign * self.k_yaw * yaw_err
        w = max(-self.max_angular, min(self.max_angular, w))
        if abs(w) < self.min_pivot_w:
            w = math.copysign(self.min_pivot_w,
                              w if w != 0.0 else self.yaw_sign * yaw_err)
        return w


def main(args=None):
    rclpy.init(args=args)
    node = TagAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())   # 退出前停车
        except Exception:
            pass   # Ctrl+C 时 context 可能已销毁，停车指令发不出就算了
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
