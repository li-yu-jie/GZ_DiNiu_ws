#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 站点预设坐标 (x, y, theta_deg)
STATIONS = {
    'HOME': (0.0, 0.0, 0.0),
    'A': (1.5, 0.0, 0.0),    # 示例坐标，需根据实际地图修改
    'B': (2.0, 1.0, 90.0),
    'C': (3.0, -1.0, -90.0)
}

def euler_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0)
    )

class DiuNiuFMSNode(Node):
    def __init__(self):
        super().__init__('diuniu_fms_node')

        # 超时参数
        self.declare_parameter('visual_align_timeout', 60.0)
        self.declare_parameter('nav_timeout', 180.0)
        self.declare_parameter('nav2_active_timeout', 10.0)
        self.visual_align_timeout = self.get_parameter('visual_align_timeout').value
        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.nav2_active_timeout = self.get_parameter('nav2_active_timeout').value

        # —— 动作时序参数（2026-09-02 用户实车定案，均支持 ros2 param set 热调）——
        # 任务时序：升叉 → 导航到取货点 → 视觉矫正 → 降叉 → 后退插取 → 升叉(抬货)
        #          → 导航到卸货点 → 视觉矫正 → 后退送货 → 降叉(放货) → 前进退出 → 回停车点
        self.declare_parameter('fork_up_time', 4.0)          # 升叉时长 (s)
        self.declare_parameter('fork_down_time', 6.0)        # 降叉时长 (s)
        self.declare_parameter('insert_reverse_dist', 1.0)   # 后退插取/送货距离 (m)，现场标定
        self.declare_parameter('insert_reverse_speed', 0.15) # 后退速度 (m/s)，进出货架宜慢
        self.declare_parameter('forward_clear_dist', 1.2)    # 卸货后前进退出距离 (m)
        self.declare_parameter('forward_clear_speed', 0.2)   # 前进退出速度 (m/s)
        self.fork_up_time = self.get_parameter('fork_up_time').value
        self.fork_down_time = self.get_parameter('fork_down_time').value
        self.insert_reverse_dist = self.get_parameter('insert_reverse_dist').value
        self.insert_reverse_speed = self.get_parameter('insert_reverse_speed').value
        self.forward_clear_dist = self.get_parameter('forward_clear_dist').value
        self.forward_clear_speed = self.get_parameter('forward_clear_speed').value

        # 注意：不在构造函数里 waitUntilNav2Active()（会把节点挂死），
        # 改为任务真正开始时带超时等待，见 nav_to_station()
        self.navigator = BasicNavigator()

        # Publisher & Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.task_sub = self.create_subscription(String, '/dispatch_task', self.task_callback, 10)
        self.align_done_sub = self.create_subscription(Bool, '/tag_align_done', self.align_done_callback, 10)
        # 急停监听：/cmd_vel_joy 上 angular.x > 0.5 表示紧急停止（STM32 断电通道）。
        # 本节点自己发布的 Twist 永不设置 angular.x，因此不会误判自己的消息。
        self.e_stop_sub = self.create_subscription(Twist, '/cmd_vel_joy', self.e_stop_callback, 10)
        # 任务取消通道：/fms_cancel (std_msgs/Bool)，true 即中止当前任务并停车
        self.cancel_sub = self.create_subscription(Bool, '/fms_cancel', self.cancel_callback, 10)

        # Param client for tag_align
        self.param_client = self.create_client(SetParameters, '/tag_align/set_parameters')

        self.state = 'IDLE'
        self.current_pickup = None
        self.current_dropoff = None
        self.align_finished = False
        self.abort_requested = False

        # 任务状态机在独立工作线程执行（避免在定时器/订阅回调里嵌套 spin）。
        # 所有 BasicNavigator 调用（内部走全局 executor 的 spin_until_future_complete）
        # 只发生在此线程；本节点自身由主线程的私有 executor spin，互不冲突。
        self._stop_event = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("🚀 FMS 调度节点已启动，等待派单...")

    def publish_stop(self):
        """补发一帧全零速度（本节点永不设置 angular.x，不会触发急停链）。"""
        self.cmd_vel_pub.publish(Twist())

    def set_tag_align_enabled(self, enabled):
        # 注意：只能在工作线程调用（节点由主线程 executor spin，
        # 这里用轮询代替 spin_until_future_complete，避免嵌套 spin）
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/tag_align 参数服务不可用（5s 超时）')
            return False
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'enabled'
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = enabled
        req.parameters.append(param)
        future = self.param_client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.05)
        if future.result() is not None:
            self.get_logger().info(f'设置 tag_align enabled={enabled} 成功')
            return True
        else:
            self.get_logger().error('设置 tag_align 参数失败')
            return False

    def e_stop_callback(self, msg):
        if msg.angular.x > 0.5:
            self.request_abort('检测到急停信号 (angular.x > 0.5)')

    def cancel_callback(self, msg):
        if msg.data:
            self.request_abort('收到 /fms_cancel 取消指令')

    def request_abort(self, reason):
        if self.state == 'IDLE' or self.abort_requested:
            return
        self.get_logger().warn(f'⚠️ {reason}，立即中止当前任务！')
        self.abort_requested = True
        self.publish_stop()  # 立即补发零速帧，后续清理由工作线程完成

    def _abort_task(self):
        """急停/取消的收尾：取消 Nav2 目标、停用 tag_align、停车、任务标记失败。"""
        try:
            self.navigator.cancelTask()
        except Exception as e:
            self.get_logger().error(f'取消导航目标失败: {e}')
        try:
            self.set_tag_align_enabled(False)
        except Exception as e:
            self.get_logger().error(f'停用 tag_align 失败: {e}')
        self.publish_stop()
        self.get_logger().warn('🛑 任务已中止（急停/取消），标记为失败')
        self.state = 'IDLE'
        self.current_pickup = None
        self.current_dropoff = None
        self.abort_requested = False

    def _fail_task(self, reason):
        if self.abort_requested:
            self._abort_task()
            return
        self.get_logger().error(f'❌ 任务失败: {reason}')
        self.publish_stop()
        self.state = 'IDLE'
        self.current_pickup = None
        self.current_dropoff = None

    def task_callback(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn("当前有任务在执行，忽略新任务！")
            return

        # 解析任务，支持 "PICKUP: A, DROPOFF: B"
        # 或者 "PICKUP_COORD: x,y,yaw, DROPOFF_COORD: x,y,yaw"
        text = msg.data
        try:
            parts = text.split(',')

            # Check for coordinates format
            if "PICKUP_COORD" in text and "DROPOFF_COORD" in text:
                # payload = `PICKUP_COORD: x,y,yaw, DROPOFF_COORD: x,y,yaw`
                # Split by 'DROPOFF_COORD:'
                split2 = text.split('DROPOFF_COORD:')
                pickup_str = split2[0].replace('PICKUP_COORD:', '').strip().strip(',')
                dropoff_str = split2[1].strip()

                px, py, pyaw = map(float, pickup_str.split(','))
                dx, dy, dyaw = map(float, dropoff_str.split(','))

                self.current_pickup = (px, py, math.degrees(pyaw))
                self.current_dropoff = (dx, dy, math.degrees(dyaw))

                self.abort_requested = False
                self.state = 'FORK_UP_TRANSIT'
                self.get_logger().info(f"✅ 收到新任务！取货坐标: {self.current_pickup}, 卸货坐标: {self.current_dropoff}（先升叉再导航）")
                return

            # Legacy parsing
            pickup = parts[0].split(':')[1].strip()
            dropoff = parts[1].split(':')[1].strip()

            if pickup not in STATIONS or dropoff not in STATIONS:
                self.get_logger().error(f"未知站点: {pickup} 或 {dropoff}")
                return

            self.current_pickup = pickup
            self.current_dropoff = dropoff
            self.abort_requested = False
            self.state = 'FORK_UP_TRANSIT'
            self.get_logger().info(f"✅ 收到新任务！取货点: {pickup}, 卸货点: {dropoff}（先升叉再导航）")
        except Exception as e:
            self.get_logger().error(f"解析任务失败: {text}, 错误: {e}")

    def align_done_callback(self, msg):
        if msg.data:
            self.align_finished = True

    def wait_nav2_active(self, timeout_sec):
        """带超时等待 Nav2 激活（轮询 navigate_to_pose action server）。"""
        start_time = time.time()
        while rclpy.ok() and not self._stop_event.is_set():
            if self.abort_requested:
                return False
            if self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
                return True
            if time.time() - start_time > timeout_sec:
                return False
        return False

    def nav_to_station(self, station_or_coords):
        if isinstance(station_or_coords, tuple):
            x, y, yaw_deg = station_or_coords
            station_name = f"COORD({x:.2f}, {y:.2f})"
        else:
            x, y, yaw_deg = STATIONS[station_or_coords]
            station_name = station_or_coords

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        qx, qy, qz, qw = euler_to_quaternion(yaw_deg)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.get_logger().info(f"导航前往 {station_name} 粗定位点...")
        if not self.wait_nav2_active(self.nav2_active_timeout):
            self.get_logger().error(f'Nav2 未在 {self.nav2_active_timeout}s 内激活，任务失败！')
            return False

        try:
            self.navigator.goToPose(goal_pose)

            start_time = time.time()
            while not self.navigator.isTaskComplete():
                if self.abort_requested or self._stop_event.is_set():
                    self.navigator.cancelTask()
                    return False
                if time.time() - start_time > self.nav_timeout:
                    self.get_logger().error(f'导航到 {station_name} 超时（>{self.nav_timeout}s）！')
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.1)

            result = self.navigator.getResult()
        except Exception as e:
            self.get_logger().error(f'导航到 {station_name} 异常: {e}')
            return False

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"成功到达 {station_name} 粗定位点！")
            return True
        else:
            self.get_logger().error(f"导航到 {station_name} 失败！")
            return False

    def do_visual_align(self):
        self.get_logger().info("启动视觉对齐...")
        self.align_finished = False
        if not self.set_tag_align_enabled(True):
            return False

        succeeded = False
        try:
            # 等待对齐完成（带超时）
            start_time = time.time()
            while not self.align_finished and rclpy.ok():
                if self.abort_requested or self._stop_event.is_set():
                    self.get_logger().warn('视觉对齐被中止')
                    return False
                if time.time() - start_time > self.visual_align_timeout:
                    self.get_logger().error(f'视觉对齐超时（>{self.visual_align_timeout}s）！')
                    return False
                time.sleep(0.1)

            self.get_logger().info("视觉对齐完成！")
            succeeded = True
            return True
        finally:
            # 关闭视觉对齐，防止误触发
            if not self.set_tag_align_enabled(False):
                if succeeded:
                    self.get_logger().warn('视觉对齐已完成，但停用 tag_align 失败')

    def control_fork(self, up=True, duration=None):
        """升/降货叉，持续 duration 秒（默认按 up/down 取各自参数）。"""
        if duration is None:
            duration = self.fork_up_time if up else self.fork_down_time
        self.get_logger().info(f"执行货叉动作: {'上升' if up else '下降'} {duration:.1f}s")
        twist = Twist()
        twist.linear.z = 1.0 if up else -1.0

        try:
            start_time = time.time()
            while time.time() - start_time < duration and rclpy.ok():
                if self.abort_requested or self._stop_event.is_set():
                    self.get_logger().warn('货叉动作被中止')
                    return False
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            self.get_logger().info("货叉动作完成")
            return True
        finally:
            # 停止货叉（退出时必补零速帧）
            self.publish_stop()

    def move_timed(self, linear_x, duration, label):
        """以固定速度直行 duration 秒（盲走，无反馈；负值=倒车）。"""
        self.get_logger().info(f"{label}: v={linear_x:+.2f} m/s × {duration:.1f}s ...")
        twist = Twist()
        twist.linear.x = linear_x

        try:
            start_time = time.time()
            while time.time() - start_time < duration and rclpy.ok():
                if self.abort_requested or self._stop_event.is_set():
                    self.get_logger().warn(f'{label}被中止')
                    return False
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            self.get_logger().info(f"{label}完成")
            return True
        finally:
            # 停车（退出时必补零速帧）
            self.publish_stop()

    def reverse_insert(self, label):
        """后退插取/送货：按参数化距离和速度倒车。"""
        duration = self.insert_reverse_dist / max(self.insert_reverse_speed, 0.01)
        return self.move_timed(-abs(self.insert_reverse_speed), duration, label)

    def forward_clear(self):
        """卸货后前进退出货架。"""
        duration = self.forward_clear_dist / max(self.forward_clear_speed, 0.01)
        return self.move_timed(abs(self.forward_clear_speed), duration,
                               f'底盘前进 {self.forward_clear_dist:.1f}m 退出')

    def _worker_loop(self):
        """任务状态机主循环（工作线程，回调上下文之外，允许服务调用轮询等待）。"""
        while not self._stop_event.is_set() and rclpy.ok():
            if self.state == 'IDLE':
                self._stop_event.wait(0.2)
                continue
            if self.abort_requested:
                self._abort_task()
                continue
            try:
                self.fms_step()
            except Exception as e:
                self.get_logger().error(f'任务状态机异常: {e}')
                self._fail_task(f'状态机异常: {e}')

    def fms_step(self):
        # 任务时序（2026-09-02 用户实车定案，修正旧版"升叉→前进"方向颠倒问题）：
        #   升叉 → 导航取货点 → 视觉矫正 → 降叉 → 后退插取 → 升叉(抬货)
        #   → 导航卸货点 → 视觉矫正 → 后退送货 → 降叉(放货) → 前进退出 → 回停车点
        if self.state == 'FORK_UP_TRANSIT':
            if self.control_fork(up=True):
                self.state = 'NAV_TO_PICKUP'
            else:
                self._fail_task('发车前升叉失败')

        elif self.state == 'NAV_TO_PICKUP':
            if self.nav_to_station(self.current_pickup):
                self.state = 'VISION_ALIGN_PICKUP'
            else:
                self._fail_task('导航到取货点失败')

        elif self.state == 'VISION_ALIGN_PICKUP':
            if self.do_visual_align():
                self.state = 'FORK_DOWN_PRE_INSERT'
            else:
                self._fail_task('取货点视觉对齐失败')

        elif self.state == 'FORK_DOWN_PRE_INSERT':
            if self.control_fork(up=False):
                self.state = 'REVERSE_INSERT'
            else:
                self._fail_task('插取前降叉失败')

        elif self.state == 'REVERSE_INSERT':
            if self.reverse_insert('后退插取货物'):
                self.state = 'FORK_UP_LOAD'
            else:
                self._fail_task('后退插取失败')

        elif self.state == 'FORK_UP_LOAD':
            if self.control_fork(up=True):
                self.state = 'NAV_TO_DROPOFF'
            else:
                self._fail_task('抬货升叉失败')

        elif self.state == 'NAV_TO_DROPOFF':
            if self.nav_to_station(self.current_dropoff):
                self.state = 'VISION_ALIGN_DROPOFF'
            else:
                self._fail_task('导航到卸货点失败')

        elif self.state == 'VISION_ALIGN_DROPOFF':
            if self.do_visual_align():
                self.state = 'REVERSE_PLACE'
            else:
                self._fail_task('卸货点视觉对齐失败')

        elif self.state == 'REVERSE_PLACE':
            if self.reverse_insert('后退送卸货物'):
                self.state = 'FORK_DOWN_UNLOAD'
            else:
                self._fail_task('后退送货失败')

        elif self.state == 'FORK_DOWN_UNLOAD':
            if self.control_fork(up=False):
                self.state = 'FORWARD_CLEAR'
            else:
                self._fail_task('卸货降叉失败')

        elif self.state == 'FORWARD_CLEAR':
            if self.forward_clear():
                self.state = 'NAV_TO_HOME'
            else:
                self._fail_task('卸货后前进退出失败')

        elif self.state == 'NAV_TO_HOME':
            if self.nav_to_station('HOME'):
                self.get_logger().info("🎉 整套物流任务圆满完成！返回待命。")
                self.state = 'IDLE'
                self.current_pickup = None
                self.current_dropoff = None
            else:
                self._fail_task('返回 HOME 失败')

    def shutdown(self):
        """节点关闭时让工作线程干净退出。"""
        self._stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=3.0)

def main(args=None):
    rclpy.init(args=args)
    node = DiuNiuFMSNode()
    # 用私有 executor spin 本节点（而非全局 executor），
    # 使工作线程里 BasicNavigator 内部的全局 executor spin 不与回调线程冲突
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
