#!/bin/bash
# start_apriltag.sh — 一键启动 AprilTag 识别链（宿主机或容器内均可运行）
#
# 自动定位 XW500U3 相机（不管设备号怎么漂移）并启动全链路。
# 用法：
#   ./start_apriltag.sh              # 无界面
#   ./start_apriltag.sh rviz:=true   # 带 RViz 画面
#
# 宿主机上运行 → 自动 docker exec 进容器 ros2_humble 启动；
# 容器内运行（ros2y 进来后）→ 直接本地启动。

set -eu
WS=/home/y/GZ_DiNiu_ws
SCRIPT="$WS/src/diuniu_apriltag/scripts/ensure_camera.sh"

if [ -f /.dockerenv ]; then
    # —— 容器内模式 ——
    # /dev 节点通常已存在；缺失时 mknod 需要 root，走 sudo
    DEV=$(bash "$SCRIPT" 2>/dev/null) || DEV=$(sudo bash "$SCRIPT")
    echo "相机设备: $DEV"
    # ROS setup.bash 引用未定义变量，与 set -u 冲突，先临时关掉
    set +u
    source /opt/ros/humble/setup.bash
    source "$WS/install/setup.bash"
    set -u
    exec ros2 launch diuniu_apriltag apriltag.launch.py video_device:="$DEV" "$@"
else
    # —— 宿主机模式 ——
    DEV=$(docker exec ros2_humble bash "$SCRIPT")
    echo "相机设备: $DEV"
    exec docker exec -it -e DISPLAY=:1 ros2_humble \
        su --whitelist-environment=DISPLAY - y -c \
        "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
         ros2 launch diuniu_apriltag apriltag.launch.py video_device:=$DEV $*"
fi
