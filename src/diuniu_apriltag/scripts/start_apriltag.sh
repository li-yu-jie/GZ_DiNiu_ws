#!/bin/bash
# start_apriltag.sh — 一键启动 AprilTag 识别链（三种环境自适应）
#
# 自动定位 XW500U3 相机（不管设备号怎么漂移）并启动全链路。
# 用法：
#   ./start_apriltag.sh              # 无界面
#   ./start_apriltag.sh rviz:=true   # 带 RViz 画面
#
# 环境自适应：
#   1. 容器内（ros2y 进来后）→ 直接本地启动；
#   2. 宿主机 + ros2_humble 容器在跑 → docker exec 进容器启动；
#   3. 物理机无 Docker（如 Jetson）→ 直接本地启动。

set -eu

# 从脚本位置反推工作区根目录（scripts/ -> diuniu_apriltag/ -> src/ -> WS），
# 不再硬编码 /home/y，x86 主机和 Jetson（/home/robot）通用
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
SCRIPT="$WS/src/diuniu_apriltag/scripts/ensure_camera.sh"

launch_local() {
    # 本地启动：/dev 节点通常已存在；缺失时 mknod 需要 root，走 sudo
    DEV=$(bash "$SCRIPT" 2>/dev/null) || DEV=$(sudo bash "$SCRIPT")
    echo "相机设备: $DEV"
    # ROS setup.bash 引用未定义变量，与 set -u 冲突，先临时关掉
    set +u
    source /opt/ros/humble/setup.bash
    source "$WS/install/setup.bash"
    set -u
    exec ros2 launch diuniu_apriltag apriltag.launch.py video_device:="$DEV" "$@"
}

if [ -f /.dockerenv ]; then
    # —— 模式 1：容器内 ——
    launch_local "$@"
elif command -v docker >/dev/null 2>&1 && \
     docker ps --format '{{.Names}}' 2>/dev/null | grep -qx ros2_humble; then
    # —— 模式 2：宿主机 + ros2_humble 容器 ——
    # 容器用户名/显示号可用环境变量覆盖（默认 y / :1，也可用宿主当前 DISPLAY）
    CONTAINER_USER=${ROS2_CONTAINER_USER:-y}
    DISP=${DISPLAY:-:1}
    # 逐个参数 %q 转义后拼接：直接 $* 会把多个 launch 参数粘成一个词，解析失败
    ARGS=$(printf '%q ' "$@")
    DEV=$(docker exec ros2_humble bash "$SCRIPT")
    echo "相机设备: $DEV"
    exec docker exec -it -e DISPLAY="$DISP" ros2_humble \
        su --whitelist-environment=DISPLAY - "$CONTAINER_USER" -c \
        "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && \
         ros2 launch diuniu_apriltag apriltag.launch.py video_device:=$DEV $ARGS"
else
    # —— 模式 3：物理机无 Docker（Jetson 等）——
    launch_local "$@"
fi
