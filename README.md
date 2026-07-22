# 地牛小车 Mid360 雷达自主导航与底盘驱动运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型，轴距 $1.30\text{m}$，半宽 $0.35\text{m}$）基于 **模式 A（FAST-LIO 高精直连 SLAM 定位导航）**、**模式 A-2（FAST-LIO + AMCL 地图匹配重定位）**、**多源 EKF 传感器融合导航** 与 **模式 B（AMCL 2D 纯定位导航）** 的实车部署步骤、手柄遥控配置及多终端运行指令流。

> 快速启动：已提供一键启动全部导航节点 launch 文件 `diuniu_nav_all.launch.py`，可直接替代分终端启动。

---

## 💡 核心配置与防坑调试优化

为了保证小车在狭窄通道正常定位、顺畅过弯与稳定行驶，我们实施了以下核心系统优化：

1. **二阶中点 Runge-Kutta 轮式里程计（`/wheel_odom`）**：
   - **原理**：底盘驱动节点 `diuniu_base_node` 升级为二阶中点弧线积分算法（Midpoint Runge-Kutta 2nd Order Integration），精准还原转弯时的弧线运动轨迹，消除了传统一阶欧拉积分在拐湾时的几何推算误差。
   - **独立话题**：底盘持续向专属话题 `/wheel_odom` 发布带有标准协方差矩阵的轮式里程计，静止时速度 $v_x=0, w_z=0$ 绝对为零。
2. **多源传感器 EKF 融合（`robot_localization`）**：
   - **机制**：集成官方 `robot_localization` 软件包（配置文件 [src/diuniu_nav/config/ekf.yaml](file:///home/y/GZ_DiNiu_ws/src/diuniu_nav/config/ekf.yaml)）。
   - **优势**：同时融合轮式里程计（静止 0 漂移）、FAST-LIO（空间 2D 厘米级位姿）与板载 IMU，使小车在行进中保持高精 3D 循迹，静止时绝对定死零漂移。
3. **彻底根治直道 S 形画龙（SmacPlannerHybrid + 控制前馈调优）**：
   - **底层纠偏**：删除了底盘驱动中原有的 1.35 倍人为打角前馈过冲逻辑，转向前轮精准响应运动学 atan 计算。
   - **全局直线化**：调高全局规划器打角切换惩罚 `change_penalty: 3.5` 并将路径平滑权重提升至 `w_smooth: 0.6`，强迫 Hybrid-A* 算法在长直通道内将红线规划为绝对直线。
4. **静止姿态零漂移（FAST-LIO 紧缩 + AMCL 0.5s 墙壁自动锁死）**：
   - 紧缩 FAST-LIO IMU 协方差 `gyr_cov / acc_cov: 0.01`，抑制 IMU 积分姿态漂移。
   - AMCL 设为 `recovery_alpha_fast: 0.0`（关闭随机全局撒点，保持粒子束收敛），并启用 `time_to_update_particle_filter: 0.5`（每 0.5 秒定期刷新粒子匹配），将激光点云死死锁定在 2D 地图墙壁上。
5. **显示与手柄体验优化**：
   - **RViz2 默认全屏**：配置了 `Window Geometry`（1920x1080）与 `--qwindowgeometry` 启动参数，RViz2 打开即铺满屏幕。
   - **手柄 1s 重连**：`diuniu_joy_publisher` 设备断开与读取异常捕获的重试休眠时间固定为 `1.0s`，断连后秒级自动恢复。
6. **阻尼前轮消除剧烈抖动与电机防热**：
   在 RPP 控制器中，将最小预瞄距离 `min_lookahead_dist` 设为 **`0.9` 米**，预瞄时间设为 `2.5s`。直道 0.8m/s 下有效预瞄长达 2.0 米，消除了低速转弯时前轮频繁剧烈扭摆、转向盘电机发热过烫的问题。
7. **碰撞检测虚警消除与弯道自动切外弯**：
   - 将碰撞检测前瞻预测时间 `max_allowed_time_to_collision_up_to_carrot` 从 0.6s 降至 **`0.2s`**，消除了长车身在直角弯拐弯时车身侧面投影误触墙壁触发的虚假急停。
   - 将全局 `inflation_radius` 设为 **`0.75m`**，并调大 `non_straight_penalty` 与转弯半径，强迫小车过弯时主动大弧度切外弯行驶，为车尾留足物理空气间隙。

---

## 🚀 一键启动全部导航节点（推荐）

使用一键启动文件，可在一个终端内顺序启动雷达、FAST-LIO、底盘、Nav2 与 RViz2 全屏界面：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

### 常用可选参数：

* **开启多源 EKF 传感器融合（轮式里程计 + FAST-LIO + IMU）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_ekf:=true
  ```
* **开启 AMCL 地图匹配重定位（开机位置任意，配合 2D Pose Estimate）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_relocalization:=true
  ```

---

## 🧭 选择导航或手柄遥控运行模式

### 📍 模式一：FAST-LIO SLAM 自主导航（默认模式）

在 **【终端 4】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```

### 📍 模式二：纯轮式里程计 + AMCL 定位导航（静止 100% 零漂移，CPU 占用极低）

在 **【终端 4】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```

### 📍 模式三：话题模式手柄遥控

在 **【终端 4】** 中运行：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
```

* **按键配置**：
  * **使能键**：按住 **RT 键**（部分模式下为 **LB 键**）不放，操作左摇杆（线速）和右摇杆（转向）进行遥控。
  * **原地转向**：原地（左摇杆推 0）推右摇杆，**前轮只偏转方向，车辆不驱动前行（防走车保护）**。
  * **升降货叉**：按键 **LB 键** 上升，按键 **A 键** 下降。
  * **主动急停**：按下 **B 键** 触发紧急停止，底盘断电锁死。

---

## 📊 轮式里程计与丢失一键恢复

### 1. 实时查看轮式里程计数据
```bash
ros2 topic echo /wheel_odom
```

### 2. AMCL 丢失位置时一键重定位
如果长时间运行或极端碰撞导致 AMCL 完全丢失对齐，可运行我们提供的一键重定位脚本：
```bash
bash ~/GZ_DiNiu_ws/relocalize.sh
```
运行后，用手柄慢速控制小车走过一两个拐角，粒子群会自动迅速收敛并锁死回到正确位置。
