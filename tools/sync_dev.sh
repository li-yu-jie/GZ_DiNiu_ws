#!/bin/bash
# =============================================================================
# sync_dev.sh — 把宿主机的串口设备节点 + udev 符号链接同步进 ros2_humble 容器
#
# 背景：ros2_humble 容器 /dev 是启动时的静态快照（无 udevd），后插的 USB 串口
#   （如 N10 雷达的 ttyACM0）不会自动出现，宿主机 udev 建的符号链接也没有。
#   本脚本在宿主机执行，把容器里缺的节点/链接补齐。
#
# 用法（宿主机）：
#   ./tools/sync_dev.sh            # 同步 diuniu_chassis / n10_lidar 相关节点
# 时机：新插串口设备后、或容器重启后跑一次。
# =============================================================================
set -e
CONTAINER=ros2_humble

sync_one() {
    local link=$1          # 宿主机 /dev 下的符号链接名，如 n10_lidar
    local target
    target=$(readlink "/dev/$link" 2>/dev/null) || { echo "⚠️  /dev/$link 不存在（设备未插或规则未装），跳过"; return 0; }
    local node="/dev/$target"
    local majmin
    majmin=$(stat -c '%t %T' "$node")
    local maj min
    read maj min <<< "$majmin"
    docker exec "$CONTAINER" bash -c "
        [ -e /dev/$target ] || mknod /dev/$target c $((16#$maj)) $((16#$min))
        chmod 666 /dev/$target
        ln -sfn $target /dev/$link
    "
    echo "✔ 容器内 /dev/$link -> $target (major=$((16#$maj)) minor=$((16#$min)))"
}

docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "❌ 容器 $CONTAINER 未运行"; exit 1; }

sync_one diuniu_chassis
sync_one n10_lidar
