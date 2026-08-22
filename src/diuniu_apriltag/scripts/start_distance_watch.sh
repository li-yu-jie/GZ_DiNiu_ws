#!/bin/bash
# start_distance_watch.sh — 宿主机上弹出 "AprilTag 实时距离" 窗口
#
# 显示 tag0 的实时距离/左右/上下/偏航（10Hz 刷新）。
# 前提：apriltag 识别链路已在运行（先跑 start_apriltag.sh）。
# 用法：./start_distance_watch.sh   （可随时再跑一个，互不影响）

set -eu
WS=/home/y/GZ_DiNiu_ws
DISPLAY=:1 gnome-terminal --title "AprilTag 实时距离" --geometry 75x6+100+100 -- \
    bash -c "docker exec -it ros2_humble su - y -c \
        'source /opt/ros/humble/setup.bash && python3 $WS/src/diuniu_apriltag/scripts/distance_watch.py'; \
        echo; echo '按回车关闭'; read"
