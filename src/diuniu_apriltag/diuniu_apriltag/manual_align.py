#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType

# 等待视觉对齐完成的最长时间 (s)：tag_align 丢码/相机故障时不会发 done，
# 无超时此处会永久挂起
ALIGN_TIMEOUT = 120.0


class ManualAlignTrigger(Node):
    def __init__(self):
        super().__init__('manual_align_trigger')
        self.align_done_sub = self.create_subscription(Bool, '/tag_align_done', self.align_done_callback, 10)
        self.param_client = self.create_client(SetParameters, '/tag_align/set_parameters')
        self.align_finished = False

    def set_tag_align_enabled(self, enabled):
        """设置 tag_align enabled 参数，返回 True 表示参数被接受。"""
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return False
            self.get_logger().info('等待 /tag_align 参数服务...')
        req = SetParameters.Request()
        param = Parameter()
        param.name = 'enabled'
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = enabled
        req.parameters.append(param)
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error('【配置失败】 无法设置 tag_align 参数（服务无响应）')
            return False
        if not result.results[0].successful:
            self.get_logger().error(
                f'【配置失败】 tag_align 拒绝 enabled={enabled}: {result.results[0].reason}')
            return False
        self.get_logger().info(f'【配置成功】 tag_align enabled={enabled}')
        return True

    def align_done_callback(self, msg):
        if msg.data:
            self.align_finished = True

    def run(self):
        """返回 0 表示对齐完成，1 表示失败/超时。"""
        self.get_logger().info("=========================================")
        self.get_logger().info("▶ 手动触发：启动视觉对齐...")
        self.align_finished = False
        if not self.set_tag_align_enabled(True):
            self.get_logger().error("❌ 无法开启 tag_align，中止")
            return 1

        self.get_logger().info(f"⏳ 正在等待视觉对齐完成（超时 {ALIGN_TIMEOUT:.0f}s）...")
        t0 = time.monotonic()
        while not self.align_finished and rclpy.ok():
            if time.monotonic() - t0 > ALIGN_TIMEOUT:
                self.get_logger().error(
                    f"❌ 等待对齐完成超时（{ALIGN_TIMEOUT:.0f}s），中止")
                self.set_tag_align_enabled(False)
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            return 1
        self.get_logger().info("✅ 接收到对齐完成信号！")
        self.get_logger().info("⏸ 停止视觉对齐...")
        self.set_tag_align_enabled(False)
        self.get_logger().info("=========================================")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ManualAlignTrigger()
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("手动中止...")
        node.set_tag_align_enabled(False)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
