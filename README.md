# 🚜 地牛小车 (DiuNiu) 自动驾驶与底盘控制系统总览

欢迎使用地牛小车（前驱转向 Tricycle 车型）的 ROS 2 自主导航与底盘驱动工作空间。
本工作空间采用高度模块化设计，支持 **2D (镭神 N10)** 与 **3D (大疆 Mid360)** 双雷达技术栈无缝切换，并集成了高精度底盘控制、多源传感器 EKF 融合、Nav2 极限制导（90度原地转向解锁）及 AprilTag 视觉辅助对接系统。

---

## 📦 核心功能包目录 (Packages)

为了保持文档清晰，各个子系统的详细架构、快速启动指南及参数优化记录已拆分至各个功能包内部的 `README.md` 中。**请点击下方链接查阅对应文档**：

| 功能包 | 描述说明 | 专属文档入口 |
| :--- | :--- | :--- |
| **`diuniu_n10_nav`** | **(⭐主力推荐)** 基于镭神 N10 2D 雷达 + AMCL + EKF 的导航栈。解决了静止漂移，解锁了 90 度极限转向机动性。 | 📖 [点击查看文档](src/diuniu_n10_nav/README.md) |
| **`diuniu_nav`** | 基于大疆 Mid360 3D 雷达 + FAST-LIO 的导航栈。支持 3D 空间自遮挡滤除与高精度点云匹配。 | 📖 [点击查看文档](src/diuniu_nav/README.md) |
| **`diuniu_apriltag`** | 视觉辅助系统。利用车尾相机识别地面二维码，实现毫米级的倒车对中与货叉自动对准。 | 📖 [点击查看文档](src/diuniu_apriltag/README.md) |
| **`diuniu_base`** | 底盘硬件驱动节点。集成二阶中点里程计推算、速度平滑限制、多指令源仲裁与超时防暴冲看门狗。 | *(基础驱动模块)* |
| **`betop_teleop`** | 独立的手柄遥控节点。支持线速度/角速度映射、使能键保护及一键物理急停下发。 | *(独立启动模块)* |

> [!WARNING]
> **运行时二选一冲突**：`diuniu_n10_nav` 与 `diuniu_nav` 不能同时启动！两者会争抢 `/scan` 话题和 `/map` -> `/odom` 的 TF 树发布权。

---

## ⚙️ 环境与编译规范 (极其重要)

> [!CAUTION]
> **x86 主机**：必须在 **Docker 容器（ros2_humble 等）内部**进行编译构建！
> 绝对不要在宿主机直接跑 `colcon build`，这会把宿主机绝对路径写进 `CMakeCache` 和软链接，导致容器内路径全部悬空崩溃（报错 `can't open file build/<pkg>/setup.py`）。
>
> **Jetson 物理机**（无 Docker）：直接在系统里编译即可，无此限制，部署流程见下方「🛰️ Jetson 一键部署」。

每次在新终端运行前，请先初始化 ROS 2 环境：

```bash
cd ~/GZ_DiNiu_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**推荐的编译指令：**
```bash
cd ~/GZ_DiNiu_ws
# 常规构建（Python 包为软链安装，改完 .py / launch / yaml 无需重新编译）
colcon build --symlink-install

# ⚠️ 若彻底删除了 build/ 目录全新重编，livox_ros_driver2 必须带参数：
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

---

## 🛰️ Jetson 一键部署（aarch64 物理机，无 Docker）

除 x86 + Docker 主机外，本工作区也可部署到 **Jetson（aarch64，JetPack 6 / Ubuntu 22.04）真实物理机**，不用 Docker。

**部署步骤：**

```bash
# 1. 装好 ROS2 Humble 后，克隆代码并手动补齐子模块
#    （父仓库缺 .gitmodules，需手动 clone FAST_LIO / livox_ros_driver2 / Lslidar_ROS2_driver）

# 2. 一键安装系统依赖 + ROS 依赖包 + udev 规则
./tools/jetson_setup.sh

# 3. 按脚本结尾提示完成手动步骤：
#    - 源码编译 aarch64 版 Livox-SDK2 安装到 /usr/local/lib（x86 的 .so 不能复制）
#    - Mid360 网卡配静态 IP 172.21.22.21/24（用 N10 雷达栈可跳过）
#    - 重新登录使 dialout 组生效

# 4. 清理旧产物并重新编译（x86 的 build/install 在 ARM 上无法运行）
cd ~/GZ_DiNiu_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# 5. 建议拉满性能
sudo nvpmodel -m 0 && sudo jetson_clocks
```

**与 x86 Docker 环境的差异：**

| 事项 | x86 主机 | Jetson 物理机 |
| :--- | :--- | :--- |
| 编译位置 | 必须在容器内 | 系统里直接编译 |
| udev 规则 | 宿主机装 + `tools/sync_dev.sh` 同步进容器 | 直接装 `/etc/udev/rules.d/`（`sync_dev.sh` 不需要） |
| Livox SDK | x86 预编译 .so | 必须源码编译 aarch64 版 |
| 启动脚本 | `start_apriltag.sh` 自动走容器分支 | 同一脚本自动走物理机分支（三模式自适应） |

> [!NOTE]
> `src/diuniu_apriltag/scripts/start_apriltag.sh` 会根据环境自动选择启动方式（容器内 / 宿主机+容器 / 物理机），工作区路径从脚本位置自动推导，两台机器用同一份代码即可。

---

## 📌 硬件与物理规格 (Tricycle 模型)

无论是 2D 还是 3D 导航栈，均共用以下物理边界防撞设定：

| 参数项 | 参数值 | 说明 |
| :--- | :--- | :--- |
| **车型结构** | 前轮驱动 + 前轮转向 | 运动学控制点 `base_link` 定义在后轴中心 |
| **物理轴距 ($L$)** | $1.30 \text{ m}$ | 前轮与后轴中心线距离 |
| **车身总宽 ($W$)** | $0.70 \text{ m}$ | 左右半宽各 $0.35\text{m}$ |
| **整车车长** | $1.90 \text{ m}$ (不含尾部货叉) | 车头（雷达桅杆端）$x = +1.60\text{m}$，车尾 $x = -0.30\text{m}$。<br>货叉在车身范围内伸出，整体轮廓即车身轮廓。 |
| **底盘通信** | 串口 `/dev/ttyUSB0` | 波特率 `460800` |
| **转向极限** | **$90^\circ$** | **(特权解锁)** 系统支持原地绕后轴中心点极小半径转弯！ |

---

## 🛡️ 系统安全与仲裁协定

为保证这台 1.9 米长工业 AGV 的绝对安全，整个工作空间底层遵循以下硬性协定：

1. **底层看门狗**：`diuniu_base` 底层驱动若超过 $0.2\text{s}$ 未收到速度话题，将高频发送全零数据包强行刹停底盘，坚决杜绝“丢码盲倒”及“网络卡顿冲撞”。
2. **紧急停止协议**：向 `/cmd_vel` 话题发送 `angular.x > 0.5` 为系统约定急停信号，底层单片机会触发物理级断电急停。
3. **双指令源仲裁 (手柄绝对优先)**：
   - 底盘同时监听 `/cmd_vel` (Nav2导航) 与 `/cmd_vel_joy` (手柄)。
   - **手柄按下使能键期间**，完全无视 Nav2 指令（日志显示 `🎮 [仲裁]`）。
   - **手柄松开 $0.5\text{s}$ 后**，自动平滑交还控制权给 Nav2。
4. **静默让权**：向 `/cmd_vel_joy` 发消息的控制端，在停止遥控后必须**彻底停止发送话题**（连全零帧也不许发），否则会导致 Nav2 永久被屏蔽。

---

## 🚀 常用启动命令速查

> 所有命令均假定已先执行环境初始化（见上文「⚙️ 环境与编译规范」）：
>
> ```bash
> cd ~/GZ_DiNiu_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
> ```

### 导航（二选一，严禁同时启动）

```bash
# ⭐ N10 2D 雷达栈（主力推荐）：雷达驱动 + EKF + AMCL + Nav2 一键全起
ros2 launch diuniu_n10_nav n10_nav_all.launch.py
# 雷达串口识别异常时显式指定：
ros2 launch diuniu_n10_nav n10_nav_all.launch.py serial_port:=/dev/ttyACM0

# Mid360 3D 雷达栈：Mid360 驱动 + FAST-LIO + Nav2 一键全起
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

```bash
# RViz 可视化（N10 / Mid360 栈共用同一份配置）
rviz2 -d ~/GZ_DiNiu_ws/src/diuniu_nav/rviz/diuniu_nav.rviz
```

> ℹ️ N10 栈的 AMCL 已配置**停车位自动初始位姿**（`set_initial_pose: true`，坐标 = waypoints.json 的「初始位置」），从停车位开机无需任何操作即可直接导航。若从别处开机，用 RViz 的 `2D Pose Estimate` 纠正一次即可；不给初始位姿时 Nav2 会一直等待（目标点无响应），补上后随时恢复，不存在超时睡死。Mid360 栈由 FAST-LIO 自动定位，同样无需手动标位姿。

### 建图

```bash
# N10 栈：slam_toolbox 在线建图（自带 RViz）
ros2 launch diuniu_n10_nav n10_mapping.launch.py

# Mid360 栈：FAST-LIO 建图，Ctrl+C 退出时自动落盘 PCD（再转 2D 地图）
ros2 launch diuniu_nav diuniu_mapping.launch.py
```

### 手柄遥控

```bash
# 常规：手柄 -> /cmd_vel_joy 话题（经底盘仲裁，与 Nav2 共存，推荐）
ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py

# 串口直控底盘（绕过仲裁，调试底盘专用）
ros2 launch betop_teleop diuniu_teleop_serial.launch.py port:=/dev/ttyUSB0
```

### AprilTag 视觉对接

```bash
# 一键启动识别链（自动定位 XW500U3 相机，三环境自适应：容器/宿主机+容器/Jetson）
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh rviz:=true   # 带画面

# 实时距离监视窗口
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_distance_watch.sh

# 手动触发一次完整对中（推荐，完成后自动暂停）
ros2 run diuniu_apriltag manual_align
```

### 常用排查

```bash
ros2 topic hz /scan                 # 雷达数据流是否正常（N10 应 ~10Hz）
ros2 topic echo /joy                # 手柄输入是否在线（轴应满量程 ±1.0）
ros2 run tf2_ros tf2_echo map base_link   # 定位 TF 链是否打通
```
