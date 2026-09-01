#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go_to 盲走控制器闭环仿真单元测试
"""

import unittest
import math
from Path.ackermann_odometry import AckermannOdometry
from Path.go_to import PointToPointController


class TestGoToController(unittest.TestCase):

    def test_point_to_point_simulation(self):
        """模拟在没有物理底盘情况下，死算里程计与控制器闭环向目标点收敛"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        odom.set_imu_yaw(0.0)
        
        target_x = 2.5
        target_y = 1.0
        
        controller = PointToPointController(odom, dist_tolerance=0.20)
        
        # 闭环仿真 100 步 (dt=0.05s)
        dt = 0.05
        arrived = False
        for _ in range(200):
            vx, wz, is_arrived = controller.compute_control(target_x, target_y)
            if is_arrived:
                arrived = True
                break
            
            # 利用前向运动学模拟更新里程计
            # wz 影响 yaw 变化, vx 影响位移
            curr_x, curr_y, curr_th_deg = odom.get_pose()
            new_yaw_deg = curr_th_deg + math.degrees(wz * dt)
            odom.update(vx=vx, dt=dt, raw_imu_yaw_deg=new_yaw_deg)

        self.assertTrue(arrived, "控制器未能在设定时间内收敛至目标区域")
        final_x, final_y, _ = odom.get_pose()
        dist_to_target = math.hypot(target_x - final_x, target_y - final_y)
        self.assertLessEqual(dist_to_target, 0.25)

    def test_go_to_path_simulation(self):
        """测试按多航点折线路线避障盲走"""
        odom = AckermannOdometry(init_x=0.0, init_y=0.0, init_theta_deg=0.0)
        controller = PointToPointController(odom, dist_tolerance=0.25)
        
        # 定义两段避障折线航点
        path = [(1.0, 0.0, 0.0), (1.0, 2.0, 90.0)]
        
        # 模拟运行
        dt = 0.05
        for wp in path:
            tx, ty, tth = wp
            for _ in range(300):
                vx, wz, is_arrived = controller.compute_control(tx, ty, tth)
                if is_arrived:
                    break
                curr_x, curr_y, curr_th_deg = odom.get_pose()
                new_yaw_deg = curr_th_deg + math.degrees(wz * dt)
                odom.update(vx=vx, dt=dt, raw_imu_yaw_deg=new_yaw_deg)

        final_x, final_y, _ = odom.get_pose()
        self.assertLessEqual(math.hypot(1.0 - final_x, 2.0 - final_y), 0.30)


if __name__ == '__main__':
    unittest.main()
