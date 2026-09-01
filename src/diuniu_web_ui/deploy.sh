#!/usr/bin/env bash
# =============================================================================
# deploy.sh — 前端构建 + 后端打包 + colcon 就地构建
#
# 用法（在宿主机 node18+ 与 ROS2 环境下）：
#   cd ~/GZ_DiNiu_ws/src/diuniu_web_ui && ./deploy.sh
# =============================================================================
set -e
cd "$(dirname "$0")"

BASE_WS="${DIUNIU_WS:-$HOME/GZ_DiNiu_ws}"   # 底层 ROS 工作区

echo "==> 1/4 安装前端依赖"
(cd frontend && npm install)

echo "==> 2/4 构建前端"
(cd frontend && npm run build)

echo "==> 3/4 拷贝前端产物到后端 web/"
rm -rf diuniu_web_ui/web
mkdir -p diuniu_web_ui/web
cp -r frontend/dist/* diuniu_web_ui/web/

echo "==> 4/4 colcon build"
cd "$BASE_WS"
if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
fi
colcon build --packages-select diuniu_web_ui

echo ""
echo "✅ 部署完成。启动："
echo "   source $BASE_WS/install/setup.bash"
echo "   ros2 launch diuniu_web_ui web_ui.launch.py"
echo "   浏览器访问 http://<工控机IP>:8000  初始账号 admin（初始密码见服务首次启动日志，仅打印一次）"
