# DiuNiu 🐂 ROS 2 底盘控制与仿真工作空间说明

本工作空间 `GZ_DiNiu_ws` 是地牛（DiuNiu）机器人的 ROS 2 核心开发环境，集成了**物理底盘串口通信**、**手柄双模式遥控器**以及 **RViz2 三维物理可视化模型**。

---

## 📂 模块化功能分类

整个工作空间的代码完全按照功能进行了高内聚、低耦合的模块化归类：

```text
GZ_DiNiu_ws/
├── src/
│   ├── diuniu_base/            # 1. 物理底盘硬件驱动模块（唯一串口控制中心）
│   │   ├── diuniu_base/        # Python 节点源码 (订阅 /cmd_vel, 发布 /odom, /imu/data)
│   │   ├── launch/             # 启动脚本目录
│   │   └── README.md           # 驱动包技术规格文档
│   │
│   ├── betop_teleop/           # 2. 手柄遥控器交互模块（提供直连/中转双模式可执行程序）
│   │   ├── betop_teleop/       # 独立的串口直控节点与话题控制节点源码
│   │   ├── launch/             # 串口与话题双启动脚本目录
│   │   └── README.md           # 遥控键位映射与协议文档
│   │
│   └── diuniu_description/     # 3. 三维仿真与RViz2可视化模块（1:1物理还原）
│       ├── urdf/               # 工业橙半透明车身、传感器坐标定义（URDF）
│       ├── launch/             # 自动加载 layout 启动脚本
│       ├── rviz/               # 预配置暗黑网格与 /odom 100 步轨迹视图
│       └── README.md           # URDF 尺寸物理测绘表与运行文档
```

---

## 🛠️ 三大功能包核心职责

| 功能包名称 | 模块定位 | 串口依赖状态 | ROS 2 通信接口 | 核心功能 |
| :--- | :--- | :--- | :--- | :--- |
| **`diuniu_base`** | 底盘驱动与正逆解 | **独占** `/dev/ttyACM0` | 订阅 `/cmd_vel`<br>发布 `/odom`, `/imu/data` | 实现前轮驱动前轮打角运动学解算，上传航向积分里程计与 IMU 数据。 |
| **`betop_teleop`** | 遥控控制与安全锁 | 话题模式下**不占串口**<br>直控模式下**独占串口** | 订阅 `/joy`<br>发布 `/cmd_vel`（话题模式） | 读取北通手柄摇杆与按键状态，提供硬件急停锁定保护机制。 |
| **`diuniu_description`**| URDF 仿真与可视化 | **无串口物理依赖** | 订阅 `/robot_description` | 在 RViz2 中以半透明工业橙形态渲染底盘及 IMU 姿态，支持运行轨迹回放。 |

---

## 🚀 快速启动指南

### 1. 本地工作区全量编译
请在您的物理主机终端运行：
```bash
cd ~/GZ_DiNiu_ws
# 清除历史编译缓存，避免符号链接混淆
rm -rf build/ install/ log/
# 编译所有包
colcon build
# 载入工作空间环境
source install/setup.bash
```

### 2. 联合实车运行与 RViz2 可视化
在完成编译后，打开三个终端运行以下命令：

- **终端 1：启动底盘驱动程序（发布 /odom，并锁定上帝视角）**
  ```bash
  source install/setup.bash
  ros2 launch diuniu_base diuniu_base.launch.py port:=/dev/ttyACM0
  ```
- **终端 2：启动话题模式手柄遥控（中转 /cmd_vel 话题，兼顾升降与急停）**
  ```bash
  source install/setup.bash
  ros2 launch betop_teleop diuniu_teleop_cmd_vel.launch.py
  ```
- **终端 3：启动 RViz2 3D 可视化小车**
  ```bash
  source install/setup.bash
  ros2 launch diuniu_description display.launch.py
  ```

> [!TIP]
> **直控调试模式**：若未启动 ROS 2 底盘驱动，只为测试单片机固件逻辑，可直接通过 `ros2 launch betop_teleop diuniu_teleop_serial.launch.py` 开启手柄串口直控，直接将数据封装写入单片机。
