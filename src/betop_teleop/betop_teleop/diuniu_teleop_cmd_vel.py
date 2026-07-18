#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 底盘手柄遥控节点 - 话题模式
===================================
功能：
  - 订阅 ROS 2 /joy 话题，读取手柄数据
  - 只负责发布 geometry_msgs/msg/Twist 到 /cmd_vel_joy 话题，不占用任何物理串口
  - 升降动作映射为 linear.z，急停动作映射为 angular.x
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading

class DiuNiuTeleopCmdVel(Node):
    def __init__(self):
        super().__init__('diuniu_teleop_cmd_vel')

        # ──────────────────────────────────────────
        # 声明 ROS 2 参数
        # ──────────────────────────────────────────
        self.declare_parameter('max_linear_speed', 1.2)
        self.declare_parameter('max_angular_speed', 2.5)  # 最大旋转角速度 (rad/s)
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_steer', 2)
        self.declare_parameter('enable_button', 7)
        self.declare_parameter('stop_button', 1)
        self.declare_parameter('button_lift_up', 4)
        self.declare_parameter('button_lift_down', 0)
        self.declare_parameter('steer_invert', False)
        self.declare_parameter('linear_invert', False)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('linear_deadband', 0.05)
        self.declare_parameter('angular_deadband', 0.10)

        # ──────────────────────────────────────────
        # 读取参数
        # ──────────────────────────────────────────
        self.max_speed = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_steer = self.get_parameter('axis_steer').value
        self.enable_button = self.get_parameter('enable_button').value
        self.stop_button = self.get_parameter('stop_button').value
        self.button_lift_up = self.get_parameter('button_lift_up').value
        self.button_lift_down = self.get_parameter('button_lift_down').value
        self.steer_invert = self.get_parameter('steer_invert').value
        self.linear_invert = self.get_parameter('linear_invert').value
        self.rate = self.get_parameter('publish_rate').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.angular_deadband = self.get_parameter('angular_deadband').value

        self.get_logger().info("🚀 [话题控制手柄节点] 正在启动，发布至 /cmd_vel_joy 话题...")
        self.get_logger().info(f"配置参数: 最大线速={self.max_speed} m/s, 最大角速度={self.max_angular} rad/s")

        # ──────────────────────────────────────────
        # 状态与发布者
        # ──────────────────────────────────────────
        self.latest_joy_msg = None
        self.last_joy_time = self.get_clock().now()
        self._joy_lock = threading.Lock()
        
        # 创建 Twist 话题发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        
        # 订阅 /joy
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 周期控制发布定时器
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.control_timer_callback)

    def joy_callback(self, msg):
        with self._joy_lock:
            self.latest_joy_msg = msg
            self.last_joy_time = self.get_clock().now()

    def control_timer_callback(self):
        now = self.get_clock().now()

        # ── 保护 1：信号超时 ──────────────────────────
        with self._joy_lock:
            latest = self.latest_joy_msg
            last_time = self.last_joy_time

        if latest is None:
            self.get_logger().info("等待手柄连接...", throttle_duration_sec=5.0)
            return

        if ((now - last_time).nanoseconds / 1e9 > 0.5):
            self.get_logger().warn("手柄信号丢失，发送安全停止话题！", throttle_duration_sec=3.0)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0  # 安全停止，非急停
            self.cmd_vel_pub.publish(twist)
            return

        msg = latest

        # ── 保护 2：急停按键（B 键）────────────────────
        is_stop_pressed = (len(msg.buttons) > self.stop_button
                           and msg.buttons[self.stop_button] == 1)
        if is_stop_pressed:
            self.get_logger().error("手柄触发紧急停止 (B键)！")
            twist = Twist()
            twist.angular.x = 1.0  # 急停标志
            self.cmd_vel_pub.publish(twist)
            return

        # ── 保护 3：使能键（LB/RB 键）──────────────────
        is_enabled = (len(msg.buttons) > self.enable_button
                      and msg.buttons[self.enable_button] == 1)

        if is_enabled:
            # 读取摇杆
            joy_linear = msg.axes[self.axis_linear] if len(msg.axes) > self.axis_linear else 0.0
            joy_steer  = msg.axes[self.axis_steer]  if len(msg.axes) > self.axis_steer  else 0.0

            # 映射控制量
            linear_sign = -1.0 if self.linear_invert else 1.0
            target_speed = joy_linear * self.max_speed * linear_sign
            steer_sign = -1.0 if self.steer_invert else 1.0
            target_w = joy_steer * self.max_angular * steer_sign

            if abs(target_speed) < self.linear_deadband:
                target_speed = 0.0
            if abs(target_w) < self.angular_deadband:
                target_w = 0.0

            # 升降动作 (1.0 代表上，-1.0 代表下，0.0 代表停)
            lift_val = 0.0
            if len(msg.buttons) > self.button_lift_up and msg.buttons[self.button_lift_up] == 1:
                lift_val = 1.0
            elif len(msg.buttons) > self.button_lift_down and msg.buttons[self.button_lift_down] == 1:
                lift_val = -1.0

            # 构建并发布 Twist 消息
            twist = Twist()
            twist.linear.x = target_speed
            twist.angular.z = target_w
            twist.linear.z = lift_val  # 升降
            twist.angular.x = 0.0      # 正常运行
            self.cmd_vel_pub.publish(twist)

            # 限速打印调试信息
            self.get_logger().info(
                f"[发送 Twist] Vx={target_speed:+.3f} m/s | Wz={target_w:+.3f} rad/s | Lift={lift_val:+.1f}",
                throttle_duration_sec=0.5
            )
        else:
            # 松开 LB 键，发送全零停止
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        self.get_logger().info("正在关闭话题控制手柄节点...")
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0  # 关闭时发送安全停止，非急停
            self.cmd_vel_pub.publish(twist)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DiuNiuTeleopCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
