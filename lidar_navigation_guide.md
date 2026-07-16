# 地牛小车 Mid360 雷达自主导航运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型）基于 **模式 A（高精直连 SLAM 定位导航）** 与 **模式 B（AMCL 纯定位导航）** 的实车部署步骤 and 多终端运行指令流。

---

## 💡 核心优化说明（避坑指南）

为了保证小车在狭窄通道正常定位与顺畅行驶，我们完成了以下核心优化：
1. **统一系统时钟时间戳（`time_sync_en: false`）**：由于雷达驱动已在底层统一使用工控机系统时钟为点云打时间戳，我们在 FAST-LIO 中关闭了软件同步（`time_sync_en: false`），避免了启动瞬时通信延时导致的虚假时间偏置（导致 LIO 定位死锁、不发布 `/Odometry`）。
2. **过滤地面噪点避障（`min_height: 0.15`）**：将点云切片高度下限修改为地面上 15 厘米（`min_height: 0.15`），彻底排除了地板瓷砖反光等噪点被识别为障碍物，解决了小车因为“地面积满障碍”而反复原地触发 Recovery 且无法行驶的 bug。
3. **调小避障膨胀半径（`0.30` 米）**：调小局部与全局代价地图的膨胀半径至 `0.30` 米，既能安全防撞，又能让 1.85 米长、0.7 米宽的小车在窄通道内顺利规划通过。

---

## 🚀 多终端启动指令流

在启动导航前，请确保：
1. 已切换到 `~/GZ_DiNiu_ws` 工作目录。
2. 已完成 `colcon build`（或 `rm -rf build install log && colcon build ...`）。
3. 已 `source install/setup.bash`。

> ⚠️ **不要重复启动同一个 launch 文件**，否则会出现同名节点冲突，导致 TF 或 costmap 异常。

请在 NoMachine 终端（或 SSH）中依次打开 **5 个终端窗口**，分别运行以下命令。每个终端都需要先 source 工作空间：

### 1. 启动雷达驱动
在 **【终端 1】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
* **验证**：输出 `Init lds lidar success!`，雷达开始广播数据。

### 2. 启动 SLAM 里程计（用于提供高精 `odom` -> `base_link` 位姿及转换点云）
在 **【终端 2】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py rviz:=false
```
* **验证**：输出 `IMU Initial Done`，提供高精三维惯导里程计，并且 `/Odometry` 话题满速发布。

### 3. 启动小车底盘节点（关闭底盘自身的 TF，防冲突）
在 **【终端 3】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_base diuniu_base.launch.py pub_odom_tf:=false
```

---

## 🧭 选择导航运行模式

接下来根据您的小车当前停放位置，选择运行以下两种导航模式之一：

### 📍 模式 A：直接定位导航（从建图起点开机）
如果您的小车正好停放在之前建图的**原点（起始位置和方向）**：
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```

### 📍 模式 B：AMCL 定位导航（推荐！支持任意位置开机手动对准）
如果小车停放在房间的任意非原点位置，通过模式 B 可以在 RViz 中手动重定位：
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```

---

## 🖥️ 启动可视化监控与操作指南

在 **【终端 5】** 中运行（需在 NoMachine 图形桌面环境内）：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> **注意**：如果 RViz 报 `Message Filter dropping message` 或 `Goal was rejected by server`，请先检查：
> 1. 前 4 个终端的节点都已正常运行。
> 2. 模式 B 下已使用 2D Pose Estimate 完成初始定位。
> 3. 目标点发送在空旷、已知的 free 区域。

### 1. 手动重定位对齐（仅针对 模式 B）
* 点击 RViz 界面顶部的 **`2D Pose Estimate`**（带红/橙色定位针图标）。
* 观察地图，找到小车此刻在房间中的物理位置。
* 用鼠标在地图上**点击并按住左键，顺着小车车头物理方向拉出红色箭头，松开**。
* AMCL 定位算法会将小车吸附对齐，黑色的雷达点云与 2D 地图墙壁完美重合，彩色代价地图完全亮起。

### 2. 加载 URDF 3D 车辆模型（若模型没有显示）
* 在 RViz 左侧的 `Displays` 列表中，点击 **`RobotModel`** 左侧的小三角展开。
* 确保 `Description Source` 设置为 **`Topic`**。
* 展开 `Description Topic` 下的 **`QoS`**，将 **`Durability`** 从 `Volatile` 修改为 **`Transient Local`**。
* 3D 模型便会瞬间正确绘制在原点。

### 3. 开始自主导航
* 点击 RViz 窗口顶部的 **`2D Goal Pose`** 按钮。
* 选择距离小车较远（建议 2 米开外）的空白地带，**按住左键拖拽方向并松开**。
* 局部寻迹器（RPP）会立刻规划动线，地牛小车自主安全地平稳行驶至终点！
