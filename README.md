# 地牛小车 Mid360 雷达自主导航与底盘驱动运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型，轴距 $1.30\text{m}$，半宽 $0.35\text{m}$）基于 **模式 A（FAST-LIO 高精直连 SLAM 定位导航）**、**模式 A-2（FAST-LIO + AMCL 地图匹配重定位）**、**多源 EKF 传感器融合导航** 与 **模式 B（AMCL 2D 纯定位导航）** 的实车部署步骤、手柄遥控配置及多终端运行指令流。

> 快速启动：已提供一键启动全部导航节点 launch 文件 `diuniu_nav_all.launch.py`，可直接替代分终端启动。

---

## 💡 核心配置与防坑调试优化

为了保证小车在狭窄通道正常定位、顺畅过弯与稳定行驶，我们实施了以下核心系统优化：

1. **二阶中点 Runge-Kutta 轮式里程计（`/wheel_odom`）**：
   - **原理**：底盘驱动节点 `diuniu_base_node` 升级为二阶中点弧线积分算法（Midpoint Runge-Kutta 2nd Order Integration），精准还原转弯时的弧线运动轨迹，消除了传统一阶欧拉积分在拐弯时的几何推算误差。
   - **独立话题**：底盘持续向专属话题 `/wheel_odom` 发布带有标准协方差矩阵的轮式里程计，静止时速度 $v_x=0, w_z=0$ 绝对为零。
2. **多源传感器 EKF 融合（`robot_localization`）**：
   - **机制**：集成官方 `robot_localization` 软件包（配置文件 `src/diuniu_nav/config/ekf.yaml`）。
   - **优势**：同时融合轮式里程计（静止 0 漂移）、FAST-LIO（空间 2D 厘米级位姿）与板载 IMU，使小车在行进中保持高精 3D 循迹，静止时绝对定死零漂移。
   - **启用方式**：启动时传参 `use_ekf:=true`。
3. **彻底根治直道 S 形画龙（SmacPlannerHybrid + 控制前馈调优）**：
   - **底层纠偏**：删除了底盘驱动中原有的 1.35 倍人为打角前馈过冲逻辑，转向前轮精准响应运动学 atan 计算。
   - **全局直线化**：调高全局规划器打角切换惩罚 `change_penalty: 3.5` 并将路径平滑权重提升至 `w_smooth: 0.6`，强迫 Hybrid-A* 算法在长直通道内将红线规划为绝对直线。
4. **AMCL 粒子自恢复定位纠偏（位置偏了自动修正）**：
   - 开启 `recovery_alpha_fast: 0.1` 与 `recovery_alpha_slow: 0.001`，当定位因累积误差偏移后，AMCL 自动注入新粒子搜索正确位置，几秒内自纠回正确位姿。
   - 适度放宽 `sigma_hit: 0.2`（原 0.15 过严），兼顾匹配精度与粒子收敛速度。
   - 启用 `time_to_update_particle_filter: 0.5`（每 0.5 秒定期刷新粒子匹配），将激光点云持续锁定在 2D 地图墙壁上。
5. **静止姿态零漂移（FAST-LIO 紧缩 + AMCL 0.5s 墙壁自动锁死）**：
   - 紧缩 FAST-LIO IMU 协方差 `gyr_cov / acc_cov: 0.01`，抑制 IMU 积分姿态漂移。
6. **显示与手柄体验优化**：
   - **RViz2 默认全屏**：配置了 `Window Geometry`（1920x1080）与 `--qwindowgeometry` 启动参数，RViz2 打开即铺满屏幕。
   - **手柄 1s 重连**：`diuniu_joy_publisher` 设备断开与读取异常捕获的重试休眠时间固定为 `1.0s`，断连后秒级自动恢复。
   - **手柄直推响应**：默认关闭使能键约束（`require_enable_button: false`），直接推摇杆即可控制小车，无需按住侧面按键。
   - **手柄原地驱动**：手柄模式下 `allow_pure_rotation=True`，原地推转向摇杆小车会直接响应转向行走。
7. **阻尼前轮消除剧烈抖动与电机防热**：
   在 RPP 控制器中，将最小预瞄距离 `min_lookahead_dist` 设为 **`0.9` 米**，预瞄时间设为 `2.5s`。直道 0.8m/s 下有效预瞄长达 2.0 米，消除了低速转弯时前轮频繁剧烈扭摆、转向盘电机发热过烫的问题。
8. **碰撞检测虚警消除与弯道自动切外弯**：
   - 将碰撞检测前瞻预测时间 `max_allowed_time_to_collision_up_to_carrot` 从 0.6s 降至 **`0.2s`**，消除了长车身在直角弯拐弯时车身侧面投影误触墙壁触发的虚假急停。
   - 将全局 `inflation_radius` 设为 **`0.75m`**，并调大 `non_straight_penalty` 与转弯半径，强迫小车过弯时主动大弧度切外弯行驶，为车尾留足物理空气间隙。
9. **雷达自身阴影与前货叉过滤器（`laserscan_filter`）**：
   - 启动 `laserscan_filter` 过滤节点，自动剔除 `base_link` 坐标系下长 $x \in [-0.25, 1.30]$ 米、宽 $y \in [-0.35, 0.35]$ 米区域内的雷达点云，输出干净的 `/scan_filtered` 话题。
10. **20Hz 串口看门狗（Watchdog）与到点防漂移**：
    底盘驱动 `diuniu_base_node` 引入控制看门狗机制。超过 0.2 秒没有接收到话题控制即自动锁死底盘，彻底根治到点后车辆缓慢爬行的硬件漂移问题。

---

## 🚀 一键启动全部导航节点（推荐）

使用一键启动文件，可在一个终端内顺序启动雷达、FAST-LIO、底盘、Nav2 与 RViz2 全屏界面：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

### 常用可选参数：

* **关闭 AMCL 重定位（使用静态原点，需从建图原点启动）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_relocalization:=false
  ```
* **开启多源 EKF 传感器融合（轮式里程计 + FAST-LIO + IMU）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_ekf:=true
  ```

> ⚠️ 一键启动会占用一个终端长期输出日志；后台运行请使用 `screen`/`tmux`。

---

## 🖥️ 多终端启动指令流

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
ros2 launch livox_ros_driver2 launch_ROS2/msg_MID360_launch.py
```

* **验证**：输出 `Init lds lidar success!`，雷达开始广播数据。

### 2. 启动 SLAM 里程计（提供高精 `odom` → `base_link` 位姿）

在 **【终端 2】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py rviz:=false
```

* **验证**：输出 `IMU Initial Done`，`/Odometry` 话题满速发布。

### 3. 启动小车底盘节点（关闭底盘自身的里程计，防冲突）

在 **【终端 3】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_base diuniu_base.launch.py pub_odom_tf:=false pub_odom_topic:=false
```

* **验证**：开始周期输出底盘发包日志，看门狗就绪。

---

## 🧭 选择导航或手柄遥控运行模式

### 📍 模式一 A：纯 FAST-LIO 静态原点（★ 终极推荐，开机即准）

如果每次都能从**建图原点**启动车辆：

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```

* **特点**：不需手动 `2D Pose Estimate`，开机直接定位，运行中绝对不飘。
* **前提**：车辆启动时位置和航向必须与建图原点一致。

### 📍 模式一 B：FAST-LIO + AMCL 地图匹配重定位（开机位置任意）

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false use_relocalization:=true
```

* **特点**：FAST-LIO 提供高精 `odom → base_link`，AMCL 负责地图匹配发布 `map → odom`，位置偏了会自动粒子恢复纠正。
* **初始位姿**：首次启动后在 RViz 中使用 **`2D Pose Estimate`** 给出粗略位置和朝向。

### 📍 模式二：AMCL 2D 纯定位导航

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```

### 📍 手柄遥控（需单独启动）

```bash
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
```

* **按键配置**：
  * **直接推摇杆**：左摇杆（线速）、右摇杆（转向），直接响应无需按住使能键。
  * **升降货叉**：**LB 键** 上升，**A 键** 下降。
  * **主动急停**：**B 键** 紧急停止，底盘断电锁死。

---

## 📊 轮式里程计与丢失一键恢复

### 1. 实时查看轮式里程计数据
```bash
ros2 topic echo /wheel_odom
```

### 2. AMCL 丢失位置时一键重定位
如果长时间运行或极端碰撞导致 AMCL 完全丢失对齐，可运行一键重定位脚本：
```bash
bash ~/GZ_DiNiu_ws/relocalize.sh
```
运行后，用手柄慢速控制小车走过一两个拐角，粒子群会自动迅速收敛并锁死回到正确位置。

---

## 🖥️ 启动可视化监控与操作指南

在 **【终端 5】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d ~/GZ_DiNiu_ws/install/diuniu_nav/share/diuniu_nav/rviz/diuniu_nav.rviz
```

### 1. 手动重定位对齐（针对模式二 AMCL 与模式一 B）

* 点击 RViz 界面顶部的 **`2D Pose Estimate`**。
* **关键**：点击位置必须是 **`base_link`**，即**两后轮中心连线中点**，而不是车头或雷达位置。
* 在地图上**点击并按住左键，顺着车头方向拉出红色箭头，松开**。
* 控制小车**前后移动半米**，AMCL 定位会瞬间吸附对齐。

### 2. 开始自主导航

* 点击 RViz 顶部的 **`2D Goal Pose`** 按钮。
* 选择距离小车较远（建议 2 米开外）的空白地带，**按住左键拖拽方向并松开**。
* 小车会自动规划路径并稳定行驶至目标点。
