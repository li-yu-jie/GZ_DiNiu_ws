#!/bin/bash
# start_distance_watch.sh — 弹出 "AprilTag 实时距离"（宿主机或容器内均可运行）
#
# 显示 tag0 的实时距离/左右/上下/偏航（10Hz 刷新）。
# 前提：apriltag 识别链路已在运行（先跑 start_apriltag.sh）。
#
# 宿主机上运行 → 桌面弹出一个 gnome-terminal 窗口；
# 容器内运行（ros2y 进来后）→ 直接在当前终端显示，Ctrl+C 退出。

set -eu
# 从脚本位置反推工作区根目录（scripts/ -> diuniu_apriltag/ -> src/ -> WS），
# 与 start_apriltag.sh 同一套自适应逻辑，x86 与 Jetson 通用
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)

if [ -f /.dockerenv ]; then
    # —— 容器内模式：当前终端直接跑 ——
    set +u   # ROS setup.bash 引用未定义变量，与 set -u 冲突
    source /opt/ros/humble/setup.bash
    set -u
    exec python3 "$WS/src/diuniu_apriltag/scripts/distance_watch.py"
elif ! command -v docker >/dev/null 2>&1 || \
     ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx ros2_humble; then
    # —— 物理机无 Docker（Jetson 等）：当前终端直接跑 ——
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    exec python3 "$WS/src/diuniu_apriltag/scripts/distance_watch.py"
else
    # —— 宿主机模式：弹桌面窗口 ——
    # 容器用户名/显示号可用环境变量覆盖（默认 y / 宿主当前 DISPLAY 或 :1）
    CONTAINER_USER=${ROS2_CONTAINER_USER:-y}
    DISP=${DISPLAY:-:1}
    DISPLAY=$DISP gnome-terminal --title "AprilTag 实时距离" --geometry 75x6+100+100 -- \
        bash -c "docker exec -it ros2_humble su - $CONTAINER_USER -c \
            'source /opt/ros/humble/setup.bash && python3 $WS/src/diuniu_apriltag/scripts/distance_watch.py'; \
            echo; echo '按回车关闭'; read"
fi
