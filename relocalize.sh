#!/bin/bash
# 当 AMCL 完全丢失位置时，调用此脚本强制全局重定位
# 脚本会把 5000 个粒子均匀散布到地图的每一个角落
# 然后只要你用手柄慢慢开几圈，粒子就会自动收敛到正确位置
echo "正在触发 AMCL 全局重定位，粒子将被均匀散布到整张地图..."
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}
echo "完成！请用手柄慢速驾驶一圈（走过几个有特征的拐角），AMCL 粒子会自动收敛到正确位置。"
echo "在 RViz 中可观察到绿色粒子云从散布状态逐渐聚集到小车真实位置。"
