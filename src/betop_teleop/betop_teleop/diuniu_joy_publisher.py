#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DiuNiu 手柄 Joy 话题发布节点
===========================
绕过 ROS2 的 joy_node（该节点对北通 BTP-KP20D 等手柄会出现
"sequence size exceeds remaining buffer" 解析错误），直接读取
/dev/input/js0 并发布 sensor_msgs/Joy 消息。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import struct
import os
import select
import threading


class DiuNiuJoyPublisher(Node):
    def __init__(self):
        super().__init__('diuniu_joy_publisher')

        # ──────────────────────────────────────────
        # 声明参数
        # ──────────────────────────────────────────
        self.declare_parameter('device', '/dev/input/js0')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('num_axes', 8)
        self.declare_parameter('num_buttons', 16)

        # ──────────────────────────────────────────
        # 读取参数
        # ──────────────────────────────────────────
        self.device = self.get_parameter('device').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.num_axes = self.get_parameter('num_axes').value
        self.num_buttons = self.get_parameter('num_buttons').value

        self.get_logger().info(
            f'🎮 [Joy 发布节点] 设备={self.device}, 轴数={self.num_axes}, 按键数={self.num_buttons}'
        )

        # ──────────────────────────────────────────
        # 状态与发布者
        # ──────────────────────────────────────────
        self.axes = [0.0] * self.num_axes
        self.buttons = [0] * self.num_buttons
        self._lock = threading.Lock()

        self._fd = None
        self._running = True
        self._ev_thread = None

        self.pub = self.create_publisher(Joy, '/joy', 10)
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joy)

        self._open_device()

    def _open_device(self):
        """尝试打开并监听指定的 joystick 设备。"""
        try:
            self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self._ev_thread = threading.Thread(target=self._read_events, daemon=True)
            self._ev_thread.start()
            self.get_logger().info(f'成功打开手柄设备: {self.device}')
        except OSError as e:
            self.get_logger().error(f'无法打开手柄设备 {self.device}: {e}')

    def _read_events(self):
        """后台线程：读取 Linux joystick 事件并更新轴/按键状态。"""
        JS_EVENT_SIZE = 8  # struct js_event: time(4) + value(2) + type(1) + number(1)
        while self._running and rclpy.ok():
            try:
                if self._fd is None:
                    self._open_device()
                    if self._fd is None:
                        threading.Event().wait(1.0)
                        continue

                r, _, _ = select.select([self._fd], [], [], 0.1)
                if not r:
                    continue

                data = os.read(self._fd, JS_EVENT_SIZE)
                if len(data) < JS_EVENT_SIZE:
                    continue

                # struct js_event: time(u32), value(s16), type(u8), number(u8)
                _time, value, typ, number = struct.unpack('IhBB', data)

                # 0x80 表示初始状态同步事件，屏蔽该标志位
                typ &= ~0x80

                if typ == 0x02:  # JS_EVENT_AXIS
                    with self._lock:
                        if number < self.num_axes:
                            self.axes[number] = value / 32767.0
                elif typ == 0x01:  # JS_EVENT_BUTTON
                    with self._lock:
                        if number < self.num_buttons:
                            self.buttons[number] = int(value)

            except Exception as e:
                self.get_logger().warn(f'读取手柄事件失败: {e}', throttle_duration_sec=2.0)

    def publish_joy(self):
        """定时发布 Joy 消息。"""
        with self._lock:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'joy'
            msg.axes = list(self.axes)
            msg.buttons = list(self.buttons)
        self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._ev_thread is not None:
            self._ev_thread.join(timeout=1.0)
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
        self.get_logger().info('Joy 发布节点已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiuNiuJoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
