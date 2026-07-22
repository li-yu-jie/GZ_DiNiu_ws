# 地牛小车 Mid360 雷达自主导航与底盘驱动运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型，轴距 $1.30\text{m}$，半宽 $0.35\text{m}$）基于 **模式 A-1（FAST-LIO 静态原点 SLAM 导航）**、**模式 A-2（FAST-LIO + AMCL 地图匹配重定位）**、**多源 EKF 传感器融合导航** 与 **模式 B（AMCL 2D 纯定位导航）** 的实车部署步骤、手柄遥控配置及多终端运行指令流。

> 快速启动：已提供一键启动全部导航节点 launch 文件 `diuniu_nav_all.launch.py`，可直接替代分终端启动。

---

## 💡 核心配置与防坑调试优化

为了保证小车在狭窄通道正常定位、顺畅过弯与安全行驶，我们实施了以下核心系统优化与安全对齐：

1. **二阶中点 Runge-Kutta 轮式里程计（`/wheel_odom`）与看门狗**：
   - **原理**：底盘驱动节点 `diuniu_base_node` 采用二阶中点弧线积分算法（Midpoint Runge-Kutta 2nd Order Integration），精准还原转弯时的弧线运动轨迹，消除了传统一阶欧拉积分在拐弯时的几何推算误差。
   - **独立话题**：底盘持续向专属话题 `/wheel_odom` 发布带有标准协方差矩阵的轮式里程计，静止时速度 $v_x=0, w_z=0$ 绝对为零。
   - **到点防漂移看门狗**：超过 0.2 秒未收到话题控制时，自动向串口发送速度和转向均为 0 的数据包，彻底根治到点后车辆缓慢爬行的硬件漂移问题。
   - **异常 dt 拦截**：当串口卡顿或调度延迟导致 $dt > 1.0\text{s}$ 时，自动丢弃当前异常帧并输出警告日志，防止位移冲激。

2. **多源传感器 EKF 融合（`robot_localization`）与 TF 冲突解耦**：
   - **机制**：集成官方 `robot_localization` 扩展卡尔曼滤波节点（配置文件 `src/diuniu_nav/config/ekf.yaml`）。
   - **优势**：融合轮式里程计（`/wheel_odom`，静止 0 漂移）、FAST-LIO 位姿（`/odom`）与 IMU（`/imu/data`），在空间 3D 循迹与静止防漂之间取得平衡。
   - **TF 广播解耦**：FAST-LIO C++ 节点扩展了 `publish.tf_en` 参数。开启 EKF（`use_ekf:=true`）时，系统自动关闭 FAST-LIO 的 `odom → base_link` TF 广播，改由 EKF 统一广播融合后的坐标变换，彻底解耦 TF 重复广播冲突。

3. **雷达自身阴影与前货叉三维空间过滤器（`laserscan_filter`）**：
   - **精确尺寸对齐**：雷达位于 `base_link` 前方 $1.215\text{m}$，前货叉最前端为 $1.60\text{m}$。过滤器 `laserscan_filter` 的过滤区域严格设为 $x \in [-0.25, 1.60]\text{m}$、y \in [-0.35, 0.35]\text{m}$，与 `nav2_params.yaml` 中全局/局部代价地图的物理 `footprint` 前边界 $1.60\text{m}$ 完全一致，彻底去除了货叉尖端的自扫噪点，消除了 AMCL 重定位的静态点云污染。
   - **地面噪点排除**：点云切片高度下限设为地面以上 10 厘米（`min_height: 0.10`，上限 `max_height: 1.2`），避免地板反光噪点引发的虚假障碍物。

4. **手柄遥控安全逻辑与物理限幅**：
   - **通道隔离**：手柄发布至 `/cmd_vel_joy`，与导航主通道 `/cmd_vel` 严格隔离。
   - **使能键安全保护**：默认恢复强制要求长按 **RT 使能键**（`require_enable_button: true`），防止误触摇杆走车。
   - **静止防走车**：手柄模式下静止状态（$v=0$）默认 `allow_pure_rotation=False`，推转向摇杆时前轮只偏转方向、驱动电机锁定为 0，防止原地扭摆挪车时误伤周遭。
   - **自转速度物理限幅**：在底盘驱动层为原地自转线速度增加上限保护 $v_{\text{front}} = \min(|w| \cdot L, 1.2\text{m/s})$，杜绝大角速度下驱动电机超速风险。

5. **AMCL 粒子自恢复与墙壁锁死**：
   - **动态粒子恢复**：配置 `recovery_alpha_fast: 0.1` 与 `recovery_alpha_slow: 0.001`。当车辆定位因累积误差或严重碰撞偏离时，AMCL 自动注入新粒子搜索真实位姿，几秒内完成自纠归位。
   - **似然场收敛**：`sigma_hit` 设为 `0.2`，兼顾高精匹配与收敛速度；`time_to_update_particle_filter: 0.5` 每 0.5 秒定期刷新粒子匹配，将点云锁定在 2D 地图墙壁上。

6. **彻底根治直道 S 形画龙与大拐弯路径设计**：
   - **规划器调优**：SmacPlannerHybrid 的打角切换惩罚设为 `change_penalty: 3.5`，路径平滑权重设为 `w_smooth: 0.6`，在长直通道内将全局路径拉为绝对直线。
   - **过弯切外弯**：全局膨胀半径设为 `0.75m`（`cost_scaling_factor: 3.0`），局部膨胀半径设为 `0.38m`（`cost_scaling_factor: 8.0`），且 RPP 控制器预瞄距离设为 `min_lookahead_dist: 0.9m`（预瞄时间 `2.5s`），强迫车辆大弧度切外弯过弯，为长车身与车尾留足物理空气间隙。

7. **系统默认配置统一**：
   - 底盘驱动节点与 launch 文件的串口默认路径统一为 `/dev/ttyUSB0`（波特率 `460800`）。
   - RViz2 默认 `1920x1080` 全屏视角展示，手柄重连休眠固定为 `1.0s`。

---

## 🚀 一键启动全部导航节点（推荐）

使用一键启动文件，可在一个终端内顺序启动雷达、FAST-LIO、底盘、Nav2 与代价地图层：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

### 常用可选参数：

* **开启多源 EKF 传感器融合（轮式里程计 + FAST-LIO + IMU，自动解耦 TF）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_ekf:=true
  ```
* **关闭 AMCL 地图匹配重定位（使用静态原点，需从建图原点启动）**：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_relocalization:=false
  ```

> ⚠️ 一键启动会占用一个终端长期输出日志；后台运行请使用 `screen`/`tmux`。

---

## 🧭 选择导航或手柄遥控运行模式

### 📍 模式一 A：纯 FAST-LIO 静态原点（开机即准，需建图原点启动）

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```

* **特点**：不需要手动 `2D Pose Estimate`，开机直接定位。
* **前提**：车辆启动时的物理位置和航向必须与建图原点保持一致。

### 📍 模式一 B：FAST-LIO + AMCL 地图匹配重定位（开机位置任意）

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false use_relocalization:=true
```

* **特点**：FAST-LIO 提供高精 `odom → base_link`，AMCL 负责地图匹配并动态更新 `map → odom`；定位偏离后 AMCL 会自动注入粒子自纠回来。
* **初始位姿**：首次启动后可在 RViz 中使用 **`2D Pose Estimate`** 给出粗略位置。

### 📍 模式二：AMCL 2D 纯定位导航

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```

### 📍 手柄遥控（独立启动）

```bash
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
```

* **安全控制**：按住 **RT 键**（使能键）的同时推动摇杆：左摇杆控制前进/后退，右摇杆控制转向。
* **升降货叉**：**LB 键** 上升，**A 键** 下降。
* **紧急停止**：按下 **B 键** 触发底盘硬件断电急停。

---

## 📊 轮式里程计与丢失一键恢复

### 1. 实时查看轮式里程计数据
```bash
ros2 topic echo /wheel_odom
```

### 2. AMCL 丢失位置时一键重定位
如果极端碰撞或强烈遮挡导致 AMCL 丢失对齐，运行一键重定位脚本：
```bash
bash ~/GZ_DiNiu_ws/relocalize.sh
```
手柄慢速控制小车经过一两个拐角，粒子群会自动迅速收敛并锁定回到正确位置。

---

## ⚠️ 已知局限与协议规范

1. **底盘串口无硬件时间戳**：底盘单片机数据包按接收时刻打上 ROS 系统时间戳。在极少数系统 CPU 严重过载或串口卡顿 $dt > 1.0\text{s}$ 时，底盘驱动会自动拦截丢帧并输出 WARNING 日志，避免积分冲激。
2. **AMCL 运动模型近似**：针对 Tricycle 前驱三轮车模型，AMCL 采用了 ROS 2 官方通用的 `DifferentialMotionModel` 差速运动模型进行近似拟合。
3. **底盘急停协议约定**：话题 `/cmd_vel` 中的 `angular.x > 0.5` 被约定为底盘紧急停止信号，其他控制节点切勿往 `angular.x` 写入非零数值。
