# 地牛小车 (DiuNiu) Mid360 雷达自主导航与底盘驱动系统

本项目为地牛小车（前驱转向 Tricycle 车型）的 ROS 2 自主导航与底盘驱动系统。系统集成了 **Livox Mid360 3D 激光雷达**、**FAST-LIO 3D SLAM**、**AMCL 粒子滤波重定位**、**robot_localization 多源 EKF 融合** 及 **Nav2 路径规划与控制**。

---

## 📌 硬件与物理规格

| 参数项 | 参数值 | 说明 |
| :--- | :--- | :--- |
| **车型结构** | Tricycle (前轮驱动 + 前轮转向) | 运动学控制点 `base_link` 定义在后轴中心 |
| **物理轴距 ($L$)** | $1.30 \text{ m}$ | 前轮与后轴中心线距离 |
| **车身半宽 ($W/2$)** | $0.35 \text{ m}$ | 车身物理总宽 $0.70 \text{ m}$ |
| **整车车长** | $1.85 \text{ m}$ | 车尾 $x = -0.25\text{m}$，前货叉尖端 $x = 1.60\text{m}$ |
| **传感器配置** | Livox Mid360 3D 雷达 | 位于 `base_link` 前方 $1.215\text{m}$、上方 $0.60\text{m}$ |
| **底盘通信** | 虚拟串口 `/dev/ttyUSB0` | 波特率 `460800` |

---

## 🚀 快速启动指南

### 1. 环境准备

每次在新终端运行前，请先初始化 ROS 2 环境：

```bash
cd ~/GZ_DiNiu_ws
source /opt/ros/humble/setup.bash  # 根据具体 ROS2 发行版 source
source install/setup.bash
```

### 2. 一键启动全部导航节点（推荐）

一键启动包含雷达驱动、FAST-LIO SLAM、底盘驱动及 Nav2 导航：

```bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

#### 常用可选启动参数

| 参数名 | 默认值 | 作用说明 |
| :--- | :--- | :--- |
| `use_relocalization` | `true` | `true`: 开启 AMCL 2D 地图匹配重定位；`false`: 使用建图原点静态定位 |
| `use_ekf` | `false` | `true`: 开启多源 EKF 融合（自动解耦 FAST-LIO 的 TF 广播冲突） |

**命令示例**：
```bash
# 启动 EKF 多源传感器融合模式
ros2 launch diuniu_nav diuniu_nav_all.launch.py use_ekf:=true

# 关闭 AMCL，使用建图原点静态原点模式
ros2 launch diuniu_nav diuniu_nav_all.launch.py use_relocalization:=false
```

---

## 🖥️ RViz2 可视化与交互指南

在新的终端窗口中运行以下命令打开预设界面：

```bash
ros2 run rviz2 rviz2 -d src/diuniu_nav/rviz/diuniu_nav.rviz
```

### 操作说明

1. **手动 2D 重定位 (`2D Pose Estimate`)**：
   - 点击顶部工具栏的 **`2D Pose Estimate`**。
   - 在地图上点击小车后轴中心点（`base_link`），按住左键拖出车头朝向箭头后松开。
   - 适当控制小车前后移动，AMCL 粒子会自动吸附到真实墙壁。
2. **设置导航目标点 (`2D Goal Pose`)**：
   - 点击顶部工具栏的 **`2D Goal Pose`**。
   - 在目标空旷区域点击并拖出目标朝向箭头，小车将自动规划路径并行驶。

---

## 🧭 运行模式说明

### 模式一：FAST-LIO SLAM 自主导航

- **模式 A-1：纯 FAST-LIO 静态原点（开机即准，需在原点启动）**
  ```bash
  ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
  ```
- **模式 A-2：FAST-LIO + AMCL 地图匹配（开机位置任意）**
  ```bash
  ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false use_relocalization:=true
  ```

### 模式二：AMCL 2D 纯定位导航

```bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```

### 模式三：手柄遥控（独立终端启动）

```bash
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
```

- **安全按键规则**：
  - **使能按键**：按住 **RT 键**，配合左摇杆（线速度 $v$）与右摇杆（转向角速度 $\omega$）进行移动。
  - **静止防走车**：静止状态下推转向摇杆仅转动前轮打角，驱动电机不输出速度。
  - **升降货叉**：**LB 键** 上升，**A 键** 下降。
  - **紧急停止**：按下 **B 键** 触发底盘硬件断电急停。

---

## 💡 核心系统优化与调试记录

### 1. 底盘二阶中点里程计与防漂移看门狗
- **二阶中点 Runge-Kutta 积分**：消除转弯时的几何推算误差。
- **看门狗（Watchdog）**：超过 $0.2\text{s}$ 未收到速度话题时，高频发送全零数据包锁死底盘，根治到点爬行漂移。
- **异常 $dt$ 拦截**：当串口卡顿或 $dt > 1.0\text{s}$ 时，自动截断异常位移冲激并记录 warning 日志。

### 2. 多源传感器 EKF 融合与 TF 解耦
- **配置路径**：`src/diuniu_nav/config/ekf.yaml`。
- **TF 解耦**：通过 `publish.tf_en` 参数，在开启 EKF（`use_ekf:=true`）时主动关闭 FAST-LIO 的 `odom → base_link` TF 广播，避免 TF 冲突。

### 3. 雷达自遮挡与货叉空间过滤器
- **空间范围**：剔除 `base_link` 下长 $x \in [-0.25, 1.60]\text{m}$、宽 $y \in [-0.35, 0.35]\text{m}$ 范围内的反射点，与代价地图 `footprint` 完全对齐，消除 AMCL 重定位自扫噪点污染。
- **高度下限**：切片高度设为 `min_height: 0.10`（地面以上 $10\text{cm}$），排除地面反光噪点。

### 4. 直道与拐弯路径优化
- **直道拉直**：设置 `change_penalty: 3.5` 及 `w_smooth: 0.6`，消除直线行驶时的画龙摇摆。
- **过弯切外弯**：全局膨胀半径设为 `0.75m`，预瞄距离设为 `0.9m`（预瞄时间 `2.5s`），强迫小车大弧度切外弯，留足车尾内切富余量。

---

## 🔍 实用调试与工具

### 1. 轮式里程计监控
```bash
ros2 topic echo /wheel_odom
```

### 2. AMCL 丢失位置一键重定位
```bash
bash ~/GZ_DiNiu_ws/relocalize.sh
```

---

## ⚠️ 协议规范与注意事项

1. **串口设备**：默认串口为 `/dev/ttyUSB0`（波特率 `460800`）。
2. **紧急停止协议**：`/cmd_vel` 话题中 `angular.x > 0.5` 为系统约定急停信号，其他节点切勿写入该字段。
3. **AMCL 运动模型**：对 Tricycle 前驱三轮车模型采用 ROS 2 官方通用的 `DifferentialMotionModel`（差速模型）进行拟合。
