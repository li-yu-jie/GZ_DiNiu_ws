# 地牛小车 Mid360 雷达自主导航与手柄控制运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型）基于 **模式 A（高精直连 SLAM 定位导航）** 与 **模式 B（AMCL 纯定位导航）** 的实车部署步骤、手柄遥控配置及多终端运行指令流。

---

## 💡 核心优化说明（避坑指南）

为了保证小车在狭窄通道正常定位与顺畅行驶，我们完成了以下核心优化：
1. **统一系统时钟时间戳（`time_sync_en: false`）**：由于雷达驱动已在底层统一使用工控机系统时钟为点云打时间戳，我们在 FAST-LIO 中关闭了软件同步（`time_sync_en: false`），避免了启动瞬时通信延时导致的虚假时间偏置（该偏置会导致 LIO 定位死锁、不发布 `/Odometry`）。
2. **过滤地面噪点避障（`min_height: 0.15`）**：将点云切片高度下限修改为地面上 15 厘米（`min_height: 0.15`），彻底排除了地板瓷砖反光等噪点被识别为障碍物，解决了小车因为“地面积满障碍”而反复原地触发 Recovery 且无法行驶 the bug。
3. **雷达自身阴影与前货叉三维空间过滤器（`laserscan_filter`）**：
   * **原理**：由于雷达安装位置（$x=1.25$）与前货叉（最前端达 $x=2.60$）的物理相对位置，雷达极易扫描到车身及正前方的金属货叉（或托盘），导致代价地图误报正前方障碍物锁死小车。
   * **解决**：增加 `laserscan_filter` 过滤节点。自动剔除 `base_link` 坐标系下长 $x \in [-0.25, 2.60]$ 米、宽 $y \in [-0.40, 0.40]$ 米区域内的雷达点云，并将处理后的干净数据以 `/scan_filtered` 话题发布。
   * **代价地图订阅**：局部和全局代价地图已重新启用实时避障（`obstacle_layer`），并将扫描源自动切换为过滤后的安全话题 `/scan_filtered`。
4. **调小避障膨胀半径（`0.30` 米）**：调小局部与全局代价地图的膨胀半径至 `0.30` 米，既能安全防撞，又能让 1.85 米长、0.7 米宽的小车在窄通道内顺利规划通过。
5. **手柄中转控制安全修复（非急停保护）**：
   * **问题**：原手柄控制节点在未接收到手柄数据、断开连接或节点关闭时，会自动向 `/cmd_vel` 话题发送急停标志（`angular.x = 1.0`），导致底盘断电锁死。
   * **修复**：修改手柄中转逻辑，在启动未连、信号断开或退出时，均发送常规的全零平稳停止信号（`Vx=0.0, Wz=0.0`），避免锁死底盘；仅在**主动按下 B 键**时才触发 E-STOP 急停。
   * **方向修正**：由于手柄物理摇杆映射与实车相反，在启动文件中开启了方向反转（`linear_invert: True` 与 `steer_invert: True`），使得推杆方向与小车前进/转向完全一致。

---

## 🚀 多终端启动指令流

在启动导航前，请确保：
1. 已切换到 `~/GZ_DiNiu_ws` 工作目录。
2. 已完成编译（`colcon build`）。
3. 已在各终端中 `source install/setup.bash`。

> ⚠️ **不要重复启动同一个 launch 文件**，否则会出现同名节点冲突，导致 TF 或 costmap 异常。

### 1. 启动雷达驱动
在 **【终端 1】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
* **验证**：输出 `Init lds lidar success!`，雷达开始广播数据。

### 2. 启动 SLAM 里程计（用于提供高精 `odom` -> `base_link` 位姿）
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
* **验证**：打印底盘速度反馈与积分坐标，且无 E-STOP 报警。

---

## 🧭 选择导航或手柄遥控运行模式

### 📍 模式一：AMCL 纯定位自主导航（推荐！支持任意位置开机手动对准）
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```
* **注**：该启动会自动包含 `pointcloud_to_laserscan` 与 `laserscan_filter` 过滤节点，实时清理雷达噪点。

### 📍 模式二：直连 SLAM 自主导航（从建图起点开机）
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```

### 📍 模式三：话题模式手柄遥控（无冲突，无锁死）
在需要手柄介入遥控时，在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
```
* **按键配置**：
  * **使能键**：按住 **RT 键**（部分模式下为 **LB 键**）不放，同时操作左摇杆（线速）和右摇杆（转向），即可遥控小车。
  * **升降货叉**：按键 **LB 键** 上升，按键 **A 键** 下降（对应北通手柄映射，可根据实际测试微调）。
  * **主动急停**：按下 **B 键** 触发紧急停止，底盘断电锁死。

---

## 🖥️ 启动可视化监控与操作指南

在 **【终端 5】** 中运行（需在 NoMachine 图形桌面环境内）：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 1. 手动重定位对齐（模式一 AMCL 必做）
* 点击 RViz 界面顶部的 **`2D Pose Estimate`**（红色定位箭头图标）。
* 观察地图，找到小车此刻在房间中的实际位置。
* 在地图上**点击并按住左键，顺着小车车头的实际物理方向拉出红色箭头，松开**。
* AMCL 定位算法会将小车对齐，红色的雷达点云与 2D 地图墙壁完美重合，彩色代价地图完全亮起。

### 2. 观察过滤后的雷达图层（Laserscan Filter）
* 在 RViz 左侧的 `Displays` 列表中，展开 **`LaserScan`** 属性。
* 将订阅的话题（Topic）修改为 `/scan_filtered`，您将看到车头货叉区域的多余噪点已被完全过滤，四周只保留外界真实障碍物点云。

### 3. 加载 URDF 3D 车辆模型（若模型没有显示）
* 在 RViz 左侧的 `Displays` 列表中，点击 **`RobotModel`** 左侧小三角展开。
* 确保 `Description Source` 设置为 **`Topic`**。
* 展开 `Description Topic` 下的 **`QoS`**，将 **`Durability`** 从 `Volatile` 修改为 **`Transient Local`**。
* 3D 模型便会瞬间正确绘制在原点。

### 4. 开始自主导航
* 点击 RViz 窗口顶部的 **`2D Goal Pose`** 按钮。
* 选择距离小车较远（建议 2 米开外）的空白地带，**按住左键拖拽方向并松开**。
* 局部寻迹器（RPP）会立刻规划动线，地牛小车自主安全地平稳行驶至目标点！
