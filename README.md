# 地牛小车 (DiuNiu) Mid360 雷达自主导航与底盘驱动系统

本项目为地牛小车（前驱转向 Tricycle 车型）的 ROS 2 自主导航与底盘驱动系统。系统集成了 **Livox Mid360 3D 激光雷达**、**FAST-LIO 3D SLAM**、**AMCL 粒子滤波重定位**、**robot_localization 多源 EKF 融合** 及 **Nav2 路径规划与控制**。

---

## 📌 硬件与物理规格

| 参数项                       | 参数值                         | 说明                                                             |
| :--------------------------- | :----------------------------- | :--------------------------------------------------------------- |
| **车型结构**           | Tricycle (前轮驱动 + 前轮转向) | 运动学控制点 `base_link` 定义在后轴中心                        |
| **物理轴距 ($L$)**   | $1.30 \text{ m}$             | 前轮与后轴中心线距离                                             |
| **车身半宽 ($W/2$)** | $0.35 \text{ m}$             | 车身物理总宽$0.70 \text{ m}$                                   |
| **整车车长**           | 车身与货叉物理总长 $1.90 \text{ m}$ | 车尾 $x = -0.30\text{m}$，车身与货叉前边界 $x = 1.60\text{m}$ |
| **传感器配置**         | Livox Mid360 3D 雷达           | 位于 `base_link` 前方 $1.215\text{m}$、上方 $0.60\text{m}$ |
| **底盘通信**           | 串口 `/dev/ttyUSB0`          | 波特率 `460800`                                                |

---

## 🚀 快速启动指南

### 1. 环境准备

每次在新终端运行前，请先初始化 ROS 2 环境：

```bash
cd ~/GZ_DiNiu_ws
source /opt/ros/humble/setup.bash  # 根据具体 ROS2 发行版 source
source install/setup.bash
```

### 2. 编译构建规范

```bash
# ⚠️ 必须先 cd 进工作空间！在其他目录（如家目录 ~）运行 colcon 会递归扫描整个目录树，
#    把无关包抓进来并污染当前目录（生成 build/ install/ log/）
cd ~/GZ_DiNiu_ws

# 常规构建（Python 包为软链安装，改完 .py / launch / yaml 无需重新编译）
colcon build --symlink-install

# ⚠️ 若删除了 build/ 目录全新重编，livox_ros_driver2 必须额外指定 cmake 参数
#    （否则 CMake 报 LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND）：
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

### 3. 一键启动全部导航节点（推荐）

一键启动包含雷达驱动、FAST-LIO SLAM、底盘驱动及 Nav2 导航：

```bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

#### 常用可选启动参数

| 参数名                 | 默认值    | 作用说明                                                               |
| :--------------------- | :-------- | :--------------------------------------------------------------------- |
| `use_relocalization` | `true`  | `true`: 开启 AMCL 2D 地图匹配重定位；`false`: 使用建图原点静态定位 |
| `use_ekf`            | `false` | `true`: 开启多源 EKF 融合（自动解耦 FAST-LIO 的 TF 广播冲突）        |

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
cd ~/GZ_DiNiu_ws   # ⚠️ 必须先 cd 到工作空间，相对路径才有效
ros2 run rviz2 rviz2 -d src/diuniu_nav/rviz/diuniu_nav.rviz
```


> **注意**：若在其他目录下用相对路径启动，RViz 找不到配置文件会静默打开一个仅有 Grid 的默认空配置（标题栏仍显示该路径并带 `*` 号），且工具栏没有 `Nav2 Goal` 按钮。

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
- **手柄/Nav2 自动仲裁**：手柄按住使能键期间，底盘自动屏蔽 Nav2 的 `/cmd_vel`（日志显示 `🎮 [仲裁]`），松开 $0.5\text{s}$ 后自动交还导航控制权。手柄空闲时 `cmd_vel_joy` 话题彻底静默（只发少量补停帧），不会与导航指令交错争抢。
- **⚠️ 限速提醒**：手柄满推 $1.2\text{ m/s}$ / $2.5\text{ rad/s}$，是 Nav2 导航限速（$0.6$ / $1.0$）的 2 倍以上，室内重载场景请谨慎满推。

---

## 💡 核心系统优化与调试记录

### 1. 底盘二阶中点里程计与防漂移看门狗

- **二阶中点 Runge-Kutta 积分**：消除转弯时的几何推算误差。
- **看门狗（Watchdog）**：超过 $0.2\text{s}$ 未收到速度话题时，高频发送全零数据包锁死底盘，根治到点爬行漂移。
- **异常 $dt$ 拦截**：当串口卡顿或 $dt > 1.0\text{s}$ 时，自动截断异常位移冲激并记录 warning 日志。

### 2. 多源传感器 EKF 融合与 TF 解耦

- **配置路径**：`src/diuniu_nav/config/ekf.yaml`。
- **TF 解耦**：通过 `publish.tf_en` 参数，在开启 EKF（`use_ekf:=true`）时主动关闭 FAST-LIO 的 `odom → base_link` TF 广播，避免 TF 冲突。
- **速度话题同步**：`bt_navigator` 的 `odom_topic` 由 launch 按模式自动注入——`use_ekf:=false` 时为 `/odom`（FAST-LIO 原始输出），`use_ekf:=true` 时为 `/odometry/filtered`（EKF 融合输出），保证速度估计与 TF 来源一致。

### 3. 雷达自遮挡过滤器与双级 footprint 设计

- **过滤空间范围**：剔除 `base_link` 下 $x \in [-0.35, 1.60]\text{m}$、宽 $y \in [-0.35, 0.35]\text{m}$ 范围内的反射点（车尾到车头/货叉前边界，含 $5\text{cm}$ 余量），覆盖车体、货叉及叉上载货的自遮挡。⚠️ 过滤盒必须**完全覆盖**全局 `footprint`，否则车体反射点落在 footprint 内会被标为障碍并膨胀，导致"自身碰撞"卡死。
- **车轮廓 footprint 与自遮挡过滤**（叉尖放低时低于切片下沿 $0.10\text{m}$，对感知不可见，必须靠轮廓保护）：
  - **统一 footprint 边界**：全局与局部 footprint 的前边界均修改为真实的 **`1.60m`**（整车含货叉总长 $1.60\text{m} - (-0.30\text{m}) = 1.90\text{m}$），确保路径规划与底盘防撞轮廓与物理实体完全契合。这不仅彻底释放了此前错误配置导致的车头虚占空间（此前误设为 `2.30m` 导致虚占前方 $70\text{cm}$ 通行区域而在窄道频繁卡死），同时也方便在窄道进行流畅的倒车转向与避障判断。
- **高度下限**：切片高度设为 `min_height: 0.10`（地面以上 $10\text{cm}$），排除地面反光噪点。⚠️ 叉尖（离地 $<10\text{cm}$）因此不可见，这是全局 footprint 必须覆盖前边界的根本原因。

### 4. 直道与拐弯路径优化

- **直道拉直**：设置 `change_penalty: 3.5` 及 `w_smooth: 0.5`，消除直线行驶时的画龙摇摆。
- **速度缩放预瞄**：RPP 开启 `use_velocity_scaled_lookahead_dist`，实际预瞄 $= v \times 2.5\text{s}$，钳制在 $[1.3, 1.5]\text{m}$（`min_lookahead_dist` = 物理轴距 $1.30\text{m}$ 为纯追踪稳定下限）。注意 `lookahead_dist` 固定值在此模式下不生效。
- **过弯切外弯**：全局膨胀半径设为 `0.75m`，强迫小车大弧度切外弯，留足车尾内切富余量。

---

## 🔍 实用调试与工具

### 1. 轮式里程计监控

```bash
ros2 topic echo /wheel_odom
```

### 2. AMCL 丢失位置重定位

在 RViz 中重新执行一次 `2D Pose Estimate`（见上文 RViz2 章节）即可，AMCL 粒子会重新吸附到真实墙壁。

---

## 🛠️ 常见故障排查（Troubleshooting）

### 1. 发目标点后不规划路径（Navigation 面板显示 `inactive`）

**根因：启动后迟迟未给初始位姿。** AMCL 未收到 `2D Pose Estimate` 前不广播 `map → odom` TF，而 `planner_server` 的 global_costmap 激活时会死等 map TF，导致 `lifecycle_manager_navigation` 超时后放弃激活，后续节点（bt_navigator 等）全部停在 inactive，发目标点毫无反应。

**预防**：启动导航后 **1~2 分钟内** 务必在 RViz 中完成 `2D Pose Estimate`。

**应急补救**（无需重启，手动完成激活）：

```bash
ros2 lifecycle set /planner_server configure && ros2 lifecycle set /planner_server activate
ros2 lifecycle set /behavior_server configure && ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /controller_server configure && ros2 lifecycle set /controller_server activate
ros2 lifecycle set /smoother_server configure && ros2 lifecycle set /smoother_server activate
ros2 lifecycle set /bt_navigator configure && ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /waypoint_follower configure && ros2 lifecycle set /waypoint_follower activate
```

### 2. 目标被接受但立即中止（Feedback 显示 `aborted`，车不动）

查看 bt_navigator 日志：

```bash
grep -E "ERROR|Timed out" ~/.ros/log/bt_navigator_*.log | tail
```

若出现 `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`，说明是 **工控机 CPU 过载**（4 核机器 load 长期 >20）导致 BT 的 action 应答超时。当前配置已适配高负载（[nav2_params.yaml](src/diuniu_nav/config/nav2_params.yaml)）：

| 参数                         | 原值           | 现值           | 说明                                          |
| :--------------------------- | :------------- | :------------- | :-------------------------------------------- |
| `bt_loop_duration`         | `10` (100Hz) | `100` (10Hz) | 恢复 Nav2 默认 tick 频率，消除 tick rate 告警 |
| `default_server_timeout`   | `20` ms      | `1000` ms    | 高负载下 20ms 必然超时导致目标 abort          |
| `wait_for_service_timeout` | `1000`       | `5000` ms    | 服务等待放宽                                  |

降负载手段（按收益排序）：RViz 挪到其他机器或关闭 PointCloud2 显示 > FAST-LIO `filter_size_surf/map` 调回 `0.5` > AMCL 粒子数下调（`max_particles 5000→2000`、`max_beams 120→60`）。

### 3. RViz 报 `Frame [map] does not exist`，界面空白

两种可能，按顺序排查：

1. **配置未加载**：Displays 面板只剩 Grid → RViz 启动时相对路径失效（见上文 RViz2 章节注意点），用绝对路径重启。
2. **AMCL 未定位**：配置正常但无 map TF → 说明 AMCL 还没收到初始位姿，执行一次 `2D Pose Estimate` 即可。

### 4. 快速确认导航栈各节点生命周期状态

```bash
for n in amcl planner_server behavior_server controller_server smoother_server bt_navigator; do
  printf "%-18s %s\n" $n "$(ros2 lifecycle get /$n 2>&1 | tail -1)"
done
```

正常状态下应全部为 `active [3]`。

---

## ⚠️ 协议规范与注意事项

1. **串口设备**：默认串口为 `/dev/ttyUSB0`（波特率 `460800`）。
2. **紧急停止协议**：`/cmd_vel` 话题中 `angular.x > 0.5` 为系统约定急停信号，其他节点切勿写入该字段。
3. **双指令源仲裁**：底盘同时订阅 `/cmd_vel`（Nav2）与 `/cmd_vel_joy`（手柄/Web 摇杆），采用**手柄优先**策略——最近 $0.5\text{s}$ 内有 `/cmd_vel_joy` 消息即忽略 Nav2 指令。向 `/cmd_vel_joy` 发消息的节点必须在停止控制后**彻底静默**（不得持续发全零帧），否则会永久屏蔽 Nav2。
4. **AMCL 运动模型**：对 Tricycle 前驱三轮车模型采用 ROS 2 官方通用的 `DifferentialMotionModel`（差速模型）进行拟合。
5. **AMCL 更新机制**：AMCL 只在运动超过 `update_min_d/update_min_a` 时才更新粒子滤波并广播 `map → odom`，完全静止时**不做**周期性重对齐（`nav2_amcl` 无定时更新类参数）。
6. **底盘里程计解耦**：`diuniu_base` 的 `pub_odom_tf` / `pub_odom_topic` 默认 `false`（由 FAST-LIO/EKF 提供 `/odom` 与 TF），仅无 SLAM 的纯底盘调试时才显式开启，避免双重发布冲突。
