#!/bin/bash
# start_apriltag.sh — 一键启动 AprilTag 识别链（宿主机上运行）
#
# 自动定位 XW500U3 相机（不管设备号怎么漂移）并在容器内启动全链路。
# 用法：
#   ./start_apriltag.sh              # 无界面
#   ./start_apriltag.sh rviz:=true   # 带 RViz 画面

set -eu
WS=/home/y/GZ_DiNiu_ws
DEV=$(docker exec ros2_humble bash "$WS/src/diuniu_apriltag/scripts/ensure_camera.sh")
echo "相机设备: $DEV"
exec docker exec -it -e DISPLAY=:1 ros2_humble \
    su --whitelist-environment=DISPLAY - y -c \
    "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
     ros2 launch diuniu_apriltag apriltag.launch.py video_device:=$DEV $*"
