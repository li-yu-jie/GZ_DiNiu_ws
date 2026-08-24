#!/bin/bash
# EKF 融合 BNO085 航向验证脚本（容器内运行）
source /opt/ros/humble/setup.bash
source /home/y/GZ_DiNiu_ws/install/setup.bash

# 1. 底盘节点
ros2 launch diuniu_base diuniu_base.launch.py > /tmp/ekf_test_base.log 2>&1 &
BASE_PID=$!

# 2. 静态 TF
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat /home/y/GZ_DiNiu_ws/src/diuniu_description/urdf/diuniu.urdf)" \
  > /tmp/ekf_test_rsp.log 2>&1 &
RSP_PID=$!

# 3. EKF
ros2 run robot_localization ekf_node --ros-args \
  --params-file /home/y/GZ_DiNiu_ws/src/diuniu_nav/config/ekf.yaml \
  > /tmp/ekf_test_ekf.log 2>&1 &
EKF_PID=$!

sleep 8

echo "===== /imu/data (frame + 四元数 + 协方差首元素) ====="
timeout 4 ros2 topic echo /imu/data --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:|- " | head -20

echo ""
echo "===== /odometry/filtered (融合位姿) ====="
timeout 4 ros2 topic echo /odometry/filtered --once 2>&1 | sed -n '/^pose:/,/^twist:/p' | head -20

echo ""
echo "===== EKF 发布的 TF (odom->base_link) ====="
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|Rotation|RPY" | head -4

echo ""
echo "===== EKF 日志尾部 ====="
tail -5 /tmp/ekf_test_ekf.log

kill $BASE_PID $RSP_PID $EKF_PID 2>/dev/null
sleep 1
pkill -f "diuniu_base" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "ekf_node" 2>/dev/null
echo "===== 测试进程已清理 ====="
