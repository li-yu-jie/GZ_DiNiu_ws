#!/bin/bash
# ensure_camera.sh — 在容器内定位 XW500U3 相机并确保 /dev 节点存在
#
# 背景：容器 /dev 是启动时的快照，相机重插/漂移后新设备号不会自动出现。
# 本脚本扫描 sysfs 按名字找到相机（index=0 为采集节点），缺失则 mknod 补齐。
# 需要在容器内以 root 运行（mknod 权限）：
#   docker exec ros2_humble bash .../ensure_camera.sh
# 输出：设备路径（如 /dev/video3）；找不到则退出码 1。

set -u
for v in /sys/class/video4linux/video*; do
    name=$(cat "$v/name" 2>/dev/null)
    index=$(cat "$v/index" 2>/dev/null)
    if [[ "$name" == XW500U3* && "$index" == "0" ]]; then
        n=$(basename "$v"); n=${n#video}
        dev="/dev/video$n"
        if [ ! -e "$dev" ]; then
            majmin=$(cat "$v/dev")
            mknod "$dev" c "${majmin%%:*}" "${majmin##*:}"
            chmod 666 "$dev"
        fi
        echo "$dev"
        exit 0
    fi
done
echo "ERROR: 未找到 XW500U3 相机（请检查 USB 连接）" >&2
exit 1
