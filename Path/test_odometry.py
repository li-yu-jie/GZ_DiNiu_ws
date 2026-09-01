#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path 里程计推算与几何工具单元测试
"""

import unittest
import math
from Path.location_table import LOCATION_TABLE, get_location, calc_distance_and_heading, normalize_angle_deg
from Path.ackermann_odometry import AckermannOdometry


class TestPathOdometry(unittest.TestCase):

    def test_location_table(self):
        """测试点位表索引与提取"""
        parking = get_location("parking")
        self.assertEqual(parking, (0.0, 0.0, 0.0))
        p1 = get_location(1)
        self.assertEqual(p1, (2.5, 1.0, 90.0))
        p6 = get_location(6)
        self.assertEqual(p6, (7.0, 6.0, -90.0))

    def test_calc_distance_and_heading(self):
        """测试目标距离与角度计算"""
        # 从 (0, 0) 到 (3, 4)，距离应为 5.0m
        dist, deg, rad = calc_distance_and_heading(0.0, 0.0, 3.0, 4.0)
        self.assertAlmostEqual(dist, 5.0, places=4)
        self.assertAlmostEqual(deg, math.degrees(math.atan2(4, 3)), places=4)

    def test_straight_line_odometry(self):
        """测试纯直线行驶里程计推演"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        odom.set_imu_yaw(0.0)
        
        # 以 1.0 m/s 行驶 1.0 秒 (dt=0.1s, 10步)
        for _ in range(10):
            odom.update(vx=1.0, dt=0.1)
            
        x, y, th = odom.get_pose()
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        self.assertAlmostEqual(th, 0.0, places=3)

    def test_arc_odometry(self):
        """测试带航向改变的中点弧线积分"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        odom.set_imu_yaw(0.0)
        
        # 转弯：每步线速度 1.0 m/s，Yaw 逐渐增加到 90 度
        total_steps = 10
        dt = 0.1
        for i in range(1, total_steps + 1):
            yaw_deg = (90.0 / total_steps) * i
            odom.update(vx=1.0, dt=dt, raw_imu_yaw_deg=yaw_deg)
            
        x, y, th = odom.get_pose()
        self.assertAlmostEqual(th, 90.0, places=2)
        # 弧线运动后 X 和 Y 应大于 0
        self.assertGreater(x, 0.5)
        self.assertGreater(y, 0.5)

    def test_reset_pose(self):
        """测试坐标位姿重置"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        odom.update(vx=2.0, dt=1.0, raw_imu_yaw_deg=0.0)
        
        # 重置至 1号点 (2.5, 1.0, 90.0)
        odom.reset_pose(2.5, 1.0, 90.0)
        x, y, th = odom.get_pose()
        self.assertEqual(x, 2.5)
        self.assertEqual(y, 1.0)
        self.assertEqual(th, 90.0)


if __name__ == '__main__':
    unittest.main()
