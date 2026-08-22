#!/bin/bash
# start_distance_watch.sh — 弹出 "AprilTag 实时距离"（宿主机或容器内均可运行）
#
# 显示 tag0 的实时距离/左右/上下/偏航（10Hz 刷新）。
# 前提：apriltag 识别链路已在运行（先跑 start_apriltag.sh）。
#
# 宿主机上运行 → 桌面弹出一个 gnome-terminal 窗口；
# 容器内运行（ros2y 进来后）→ 直接在当前终端显示，Ctrl+C 退出。

set -eu
WS=/home/y/GZ_DiNiu_ws

if [ -f /.dockerenv ]; then
    # —— 容器内模式：当前终端直接跑 ——
    source /opt/ros/humble/setup.bash
    exec python3 "$WS/src/diuniu_apriltag/scripts/distance_watch.py"
else
    # —— 宿主机模式：弹桌面窗口 ——
    DISPLAY=:1 gnome-terminal --title "AprilTag 实时距离" --geometry 75x6+100+100 -- \
        bash -c "docker exec -it ros2_humble su - y -c \
            'source /opt/ros/humble/setup.bash && python3 $WS/src/diuniu_apriltag/scripts/distance_watch.py'; \
            echo; echo '按回车关闭'; read"
fi
