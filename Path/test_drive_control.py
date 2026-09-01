#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DirectMoveController 接口（前进、倒车、用户规划路线）单元测试
"""

import unittest
import math
from Path.ackermann_odometry import AckermannOdometry
from Path.drive_control import DirectMoveController


class TestDirectMoveController(unittest.TestCase):

    def test_forward_move_to(self):
        """测试前进模式 move_to"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        controller = DirectMoveController(odom, dist_tolerance=0.20)
        
        # 目标点 (2.0, 0.0, 0.0)，前进 0.5m/s
        dt = 0.05
        for _ in range(100):
            # 仿真状态更新
            curr_x, curr_y, curr_th_deg = odom.get_pose()
            dist = math.hypot(2.0 - curr_x, 0.0 - curr_y)
            if dist <= 0.20:
                break
            odom.update(vx=0.5, dt=dt, raw_imu_yaw_deg=0.0)

        final_x, final_y, final_th = odom.get_pose()
        self.assertGreaterEqual(final_x, 1.8)

    def test_reverse_move_to(self):
        """测试倒车模式 move_to"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        controller = DirectMoveController(odom, dist_tolerance=0.20)
        
        # 目标点在后方 (-2.0, 0.0, 0.0)，倒车 -0.5m/s
        dt = 0.05
        for _ in range(100):
            curr_x, curr_y, curr_th_deg = odom.get_pose()
            dist = math.hypot(-2.0 - curr_x, 0.0 - curr_y)
            if dist <= 0.20:
                break
            odom.update(vx=-0.5, dt=dt, raw_imu_yaw_deg=0.0)

        final_x, final_y, final_th = odom.get_pose()
        self.assertLessEqual(final_x, -1.8)

    def test_move_to_point(self):
        """测试按点名 (例如 1号点) 自动查表移动"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        controller = DirectMoveController(odom, dist_tolerance=0.25)
        
        # 模拟走到底点名 1 (2.5, 1.0, 90.0)
        dt = 0.05
        for _ in range(100):
            curr_x, curr_y, _ = odom.get_pose()
            if math.hypot(2.5 - curr_x, 1.0 - curr_y) <= 0.25:
                break
            odom.update(vx=0.5, dt=dt, raw_imu_yaw_deg=0.0)

        # 检查是否能够识别点名 1
        from Path.location_table import get_location
        loc = get_location(1)
        self.assertEqual(loc, (2.5, 1.0, 90.0))


if __name__ == '__main__':
    unittest.main()
