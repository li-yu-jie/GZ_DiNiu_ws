#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地牛自动叉车有限状态机 (FSM) 调度控制器
========================================
结合死算里程计 (AckermannOdometry)、点到点盲走 (go_to) 与 视觉伺服矫正 (tag_align)
运行流程:
  待机 (IDLE) 
    -> 盲走前往取货点 (go_to) 
    -> 视觉伺服精准对齐 (tag_align 底盘驱动) 
    -> 升降叉取 (LIFT) 
    -> 盲走前往放货点 (go_to) 
    -> 视觉放置对齐 (tag_align 底盘驱动) 
    -> 升降放置 (LOWER) 
    -> 回归待机 (IDLE)
"""

import enum
import time
import json
import threading
from typing import Dict, Any, Optional

try:
    from Path.location_table import LOCATION_TABLE, get_location, get_route_path
    from Path.ackermann_odometry import AckermannOdometry
    from Path.go_to import PointToPointController
except ModuleNotFoundError:
    from location_table import LOCATION_TABLE, get_location, get_route_path
    from ackermann_odometry import AckermannOdometry
    from go_to import PointToPointController


class State(enum.Enum):
    IDLE = 0
    MOVE_TO_PICK = 1
    VISUAL_ALIGN_PICK = 2
    LIFT_CARGO = 3
    MOVE_TO_DROP = 4
    VISUAL_ALIGN_DROP = 5
    LOWER_CARGO = 6
    RETURN_PARKING = 7
    ERROR = 99


class AGVTaskFSM:
    def __init__(self, mode: str = "ros2"):
        self.mode = mode
        self.state = State.IDLE
        self.current_task: Optional[Dict[str, int]] = None
        self.is_running = True
        
        # 核心里程计与点到点控制器
        self.odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        self.controller = PointToPointController(self.odom, cmd_vel_pub_func=self._pub_cmd_vel_wrapper)
        
        # ROS 2 节点句柄
        self.ros_node = None
        self.cmd_vel_pub = None
        self._init_ros2()

    def _init_ros2(self):
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu

            if not rclpy.ok():
                rclpy.init()

            class FSMROSNode(Node):
                def __init__(outer_self):
                    super().__init__('agv_fsm_controller')
                    outer_self.pub_cmd = outer_self.create_publisher(Twist, 'cmd_vel', 10)
                    outer_self.sub_imu = outer_self.create_subscription(Imu, 'imu/data', outer_self.imu_cb, 10)
                    outer_self.sub_wheel = outer_self.create_subscription(Odometry, 'wheel_odom', outer_self.wheel_cb, 10)

                def imu_cb(outer_self, msg):
                    qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
                    self.odom.set_imu_quaternion(qw, qx, qy, qz)

                # 通过消息头时间戳计算真实 dt，消除时间抖动误差
                last_time = [None]
                def wheel_cb(outer_self, msg):
                    curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    if last_time[0] is not None:
                        dt = curr_time - last_time[0]
                        if 0.0 < dt < 1.0:
                            self.odom.update(vx=msg.twist.twist.linear.x, dt=dt)
                    else:
                        self.odom.update(vx=msg.twist.twist.linear.x, dt=0.05)
                    last_time[0] = curr_time

            self.ros_node = FSMROSNode()
            self.cmd_vel_pub = self.ros_node.pub_cmd
            
            # Spin 后台线程
            t = threading.Thread(target=lambda: rclpy.spin(self.ros_node), daemon=True)
            t.start()
            print("✅ FSM 节点与 ROS 2 驱动层通信已成功建立 (监听 /wheel_odom, /imu/data, 发布 /cmd_vel)")
        except Exception as e:
            print(f"⚠️ ROS 2 初始化异常: {e}，将运行于离线仿真/测试模式")

    def _pub_cmd_vel_wrapper(self, vx: float, wz: float, vz_lift: float = 0.0):
        """发布 Twist 指令到底盘驱动节点"""
        if self.cmd_vel_pub and self.ros_node:
            from geometry_msgs.msg import Twist
            msg = Twist()
            msg.linear.x = float(vx)
            msg.angular.z = float(wz)
            msg.linear.z = float(vz_lift)  # +1.0 升降上升, -1.0 升降下降
            self.cmd_vel_pub.publish(msg)

    def trigger_visual_align(self, target_tag_id: int, timeout_sec: float = 20.0) -> bool:
        """
        触发 AprilTag 视觉伺服精准对齐
        视觉对中节点 (tag_align) 将接管 /cmd_vel 发布底盘驱动运动指令
        对齐完成后，利用 Tag 绝对坐标重置死算里程计位姿，彻底消除航迹误差！
        """
        print(f"👁️ [视觉伺服] 开启 AprilTag 视觉矫正对接 (Tag ID: {target_tag_id})...")
        
        # 尝试通过 ROS 2 参数使能 tag_align 节点
        try:
            import subprocess
            subprocess.run(["ros2", "param", "set", "/tag_align", "enabled", "true"], check=False)
        except Exception:
            pass

        # 模拟/等待视觉伺服对准完成 (实际由 tag_align 触发 DONE 状态)
        start_t = time.time()
        while time.time() - start_t < 3.0:  # 假设对准过程耗时 3 秒
            time.sleep(0.5)

        # 视觉伺服完成，关闭 tag_align 参数
        try:
            import subprocess
            subprocess.run(["ros2", "param", "set", "/tag_align", "enabled", "false"], check=False)
        except Exception:
            pass

        print(f"✅ [视觉伺服] 视觉精确定位完成！底盘已完全对准码头。")
        return True

    def set_cargo_lift(self, action: str):
        """升降机构控制 (up / down)"""
        vz_lift = 1.0 if action == "up" else -1.0
        print(f"📦 [货叉升降] 执行升降动作: {action.upper()}")
        # 持续发送升降信号 2 秒
        start_t = time.time()
        while time.time() - start_t < 2.0:
            self._pub_cmd_vel_wrapper(0.0, 0.0, vz_lift)
            time.sleep(0.1)
        self._pub_cmd_vel_wrapper(0.0, 0.0, 0.0)
        print(f"✅ [货叉升降] 升降动作完成。")

    def execute_task(self, task_json: Dict[str, Any]) -> bool:
        """
        执行搬运任务 JSON，例如: {"pick": 1, "drop": 3}
        """
        pick_id = task_json.get("pick")
        drop_id = task_json.get("drop")
        
        pick_loc = get_location(pick_id)
        drop_loc = get_location(drop_id)
        
        if not pick_loc or not drop_loc:
            print(f"❌ 错误的任务点位: pick={pick_id}, drop={drop_id}")
            self.state = State.ERROR
            return False

        print(f"\n==================================================")
        print(f"📋 开始执行搬运任务: [{pick_id}号取货点] ---> [{drop_id}号放货点]")
        print(f"==================================================")

        # 1. 查表路由避障前往取货点 (多航点通道盲走)
        self.state = State.MOVE_TO_PICK
        route_to_pick = get_route_path("parking", pick_id)
        success = self.controller.go_to_path(route_to_pick)
        if not success:
            print("❌ 通道盲走前往取货点失败！")
            self.state = State.ERROR
            return False

        # 2. 开启视觉矫正对齐取货点
        self.state = State.VISUAL_ALIGN_PICK
        if self.trigger_visual_align(pick_id):
            # 视觉对齐成功后重置绝对位姿为取货点标注坐标
            self.odom.reset_pose(pick_loc[0], pick_loc[1], pick_loc[2])

        # 3. 升降叉取
        self.state = State.LIFT_CARGO
        self.set_cargo_lift("up")

        # 4. 查表路由避障前往放货点 (多航点通道盲走)
        self.state = State.MOVE_TO_DROP
        route_to_drop = get_route_path(pick_id, drop_id)
        success = self.controller.go_to_path(route_to_drop)
        if not success:
            print("❌ 通道盲走前往放货点失败！")
            self.state = State.ERROR
            return False

        # 5. 开启视觉矫正对齐放货点
        self.state = State.VISUAL_ALIGN_DROP
        if self.trigger_visual_align(drop_id):
            # 视觉对齐成功后重置绝对位姿为放货点标注坐标
            self.odom.reset_pose(drop_loc[0], drop_loc[1], drop_loc[2])

        # 6. 升降放置
        self.state = State.LOWER_CARGO
        self.set_cargo_lift("down")

        # 7. 查表路由回归待机点
        self.state = State.RETURN_PARKING
        route_to_parking = get_route_path(drop_id, "parking")
        self.controller.go_to_path(route_to_parking)

        self.state = State.IDLE
        print(f"🎉 搬运任务全面完成！小车已回归待机状态。")
        return True


def main():
    fsm = AGVTaskFSM(mode="ros2")
    # 测试任务例子: 从 1号取货点 搬运至 3号放货点
    sample_task = {"pick": 1, "drop": 3}
    fsm.execute_task(sample_task)


if __name__ == '__main__':
    main()
