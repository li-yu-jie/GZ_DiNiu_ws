#!/bin/bash
# =============================================================================
# jetson_setup.sh — Jetson (aarch64, 物理机无 Docker) 一键环境部署
#
# 在 Jetson 上执行一次，完成：
#   1. 安装系统依赖 + ROS2 Humble 包
#   2. 安装 udev 规则（底盘 / N10 雷达 / XW500U3 相机）
#   3. 当前用户加入 dialout 组
# 完成后按结尾提示手动做：Livox SDK2 编译、Mid360 静态 IP、colcon build。
#
# 用法：
#   ./tools/jetson_setup.sh            # 全流程
#   ./tools/jetson_setup.sh --udev     # 只装 udev 规则（已装过依赖时）
# =============================================================================
set -euo pipefail

WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# ---- 环境检查 ---------------------------------------------------------------
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "⚠️  当前架构是 $ARCH（不是 aarch64），这脚本是给 Jetson 用的，确定继续？"
    read -rp "按回车继续，Ctrl+C 取消 " _
fi

if [ ! -d /opt/ros/humble ]; then
    echo "❌ 未检测到 /opt/ros/humble，请先安装 ROS2 Humble："
    echo "   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi

install_udev() {
    echo "==> 安装 udev 规则"
    sudo cp "$WS/src/diuniu_n10_nav/config/99-diuniu.rules" /etc/udev/rules.d/
    sudo cp "$WS/src/diuniu_apriltag/scripts/99-xw500u3.rules" /etc/udev/rules.d/
    sudo udevadm control --reload
    sudo udevadm trigger
    echo "   ✔ /dev/diuniu_chassis /dev/n10_lidar /dev/xw500u3 软链接规则已生效"

    # sudo 运行时 $USER 是 root，会把 root 加进 dialout 而实际操作员没加；
    # 优先取 SUDO_USER（sudo 前的真实用户）
    TARGET_USER=${SUDO_USER:-$USER}
    if [ "$TARGET_USER" = "root" ]; then
        echo "   ⚠️  当前以 root 运行且无法确定操作员账户，跳过 dialout 分组。"
        echo "       请手动执行：sudo usermod -aG dialout <你的用户名>"
    elif ! groups "$TARGET_USER" | grep -qw dialout; then
        echo "==> 把 $TARGET_USER 加入 dialout 组（重新登录后生效）"
        sudo usermod -aG dialout "$TARGET_USER"
    fi
}

if [ "${1:-}" = "--udev" ]; then
    install_udev
    exit 0
fi

# ---- 1. 系统依赖 ------------------------------------------------------------
echo "==> 安装系统依赖"
sudo apt update
sudo apt install -y \
    python3-serial python3-numpy python3-pil python3-flask \
    libpcl-all-dev libpcap-dev libapr1-dev libeigen3-dev \
    build-essential cmake git v4l-utils

# ---- 2. ROS2 包 -------------------------------------------------------------
echo "==> 安装 ROS2 Humble 依赖包"
sudo apt install -y \
    ros-humble-usb-cam \
    ros-humble-image-proc \
    ros-humble-apriltag-ros \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-pcl-ros \
    ros-humble-joy

# ---- 3. udev 规则 -----------------------------------------------------------
install_udev

# ---- 4. 收尾提示 ------------------------------------------------------------
cat <<EOF

✅ 自动部分完成。还需手动做以下几件事：

  1. 【Livox SDK2（aarch64）】fast_lio / livox_ros_driver2 依赖它：
       git clone https://github.com/Livox-SDK/Livox-SDK2.git
       cd Livox-SDK2 && mkdir build && cd build
       cmake .. && make -j\$(nproc) && sudo make install
     （装完确认 /usr/local/lib/liblivox_lidar_sdk_shared.so 存在）

  2. 【Mid360 静态 IP】给接雷达的网卡配 172.21.22.21/24，例如：
       sudo nmcli con add type ethernet ifname eth0 ipv4.method manual \
            ipv4.addresses 172.21.22.21/24
     （用 N10 雷达栈则可跳过此项）

  3. 【编译工作区】
       cd $WS
       rm -rf build install log        # 清掉 x86 旧产物
       source /opt/ros/humble/setup.bash
       colcon build --symlink-install

  4. 【性能】建议拉满：
       sudo nvpmodel -m 0 && sudo jetson_clocks

  5. 重新登录（dialout 组生效）后插好设备，检查：
       ls -l /dev/diuniu_chassis /dev/n10_lidar /dev/xw500u3

EOF
