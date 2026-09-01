# 🐂 地牛阿克曼底盘无导航绝对坐标里程计 (Ackermann Odometry)

本模块位于 `Path/` 文件夹下，专为阿克曼底盘地牛（自动叉车）设计，用于在**弃用 Nav2 / 全局路径规划与 SLAM** 条件下，提供基于“编码器线速度/位移 + IMU 绝对航向角”的高精度航迹推算（Dead Reckoning）。

---

## 📐 1. 核心推算原理

1. **绝对航向角由 IMU 直接约束**：
   - 传统轮式里程计依赖角速度（或左右轮差速）积分，累计角度误差极大。
   - 本模块直接读取 IMU（如 BNO085）输出的绝对四元数/航向角 $\Theta_{imu}$，从根本上消除了角度积分漂移。

2. **位置中点弧线积分 (Midpoint Runge-Kutta 2nd Order)**：
   - 给定采样间隔 $\Delta t$ 内由编码器测得的驱动轮线速度 $v$（或步进位移 $\Delta s = v \cdot \Delta t$），结合前后两帧航向角的中点角 $\theta_{mid}$：
     $$\theta_{mid} = \theta_{k-1} + \frac{1}{2}(\theta_k - \theta_{k-1})$$
     $$\Delta x = \Delta s \cdot \cos(\theta_{mid})$$
     $$\Delta y = \Delta s \cdot \sin(\theta_{mid})$$
     $$X_k = X_{k-1} + \Delta x$$
     $$Y_k = Y_{k-1} + \Delta y$$

---

## 🗂️ 2. 文件组成

| 文件名 | 说明 |
| :--- | :--- |
| [location_table.py](file:///home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/Path/location_table.py) | 定义 1~6 号作业点位及待机位坐标 `LOCATION_TABLE`，以及几何计算工具 |
| [ackermann_odometry.py](file:///home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/Path/ackermann_odometry.py) | 核心里程计推算类 `AckermannOdometry`（纯 Python 依赖，高内聚） |
| [odom_manual_test.py](file:///home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/Path/odom_manual_test.py) | 实时推车闭环测试控制台程序（支持 ROS 2、串口直连和模拟模式） |
| [test_odometry.py](file:///home/y/computer/Docker/ubuntu22-rootfs/home/y/GZ_DiNiu_ws/Path/test_odometry.py) | 数学与逻辑单元测试脚本 |

---

## 📍 3. 作业点位表 (`LOCATION_TABLE`)

```python
LOCATION_TABLE = {
    "parking": (0.0, 0.0, 0.0),
    1: (2.5, 1.0, 90.0),   # 1号取货点
    2: (4.0, 1.0, 90.0),   # 2号取货点
    3: (2.5, 6.0, -90.0),  # 3号放货点
    4: (4.0, 6.0, -90.0),  # 4号放货点
    5: (5.5, 6.0, -90.0),  # 5号放货点
    6: (7.0, 6.0, -90.0)   # 6号放货点
}
```

---

## 🚀 4. 手动推车闭环测试步骤

### 运行测试脚本

1. **模拟模式 (无硬件，快速验证终端显示与逻辑)**：
   ```bash
   python3 Path/odom_manual_test.py --mode sim
   ```

2. **ROS 2 模式 (订阅 `/imu/data` 与 `/wheel_odom`)**：
   ```bash
   python3 Path/odom_manual_test.py --mode ros2
   ```

3. **底盘串口直连模式 (连接物理串口解析 52 字节二进制帧)**：
   ```bash
   python3 Path/odom_manual_test.py --mode serial --port /dev/ttyUSB0 --baud 460800
   ```

### 交互快捷键

- `r`: 将当前位姿重置为原点 `(0.000, 0.000, 0.0°)`
- `1` ~ `6`: 将当前位姿校准重置为对应的作业点位坐标
- `c`: 清空累计行驶里程
- `q`: 退出测试程序

### 精度闭环验证推荐流程

1. **直线位移校验**：在待机位按 `r` 重置原点，拉线或用卷尺手动推车直线行走 `2.0` 米，检查终端显示的 `X` 是否在 `2.000 ± 0.03m` 范围内。
2. **角度与转向校验**：手动旋转车辆 90°，检查终端 `Theta` 是否精准显示为 `90.0°` 或 `-90.0°`（IMU 无角度漂移）。
3. **矩形闭环校验**：手动将车推行 2m × 2m 矩形一圈回到起点，验证终点 `(X, Y)` 坐标闭环精度。

## 🛣️ 6. 极简直接调用接口 (`Path/drive_control.py`)

如果您希望**完全自己规划路线**，只需直接调用以下两个函数：

### 单点移动函数 `move_to`

```python
from Path.drive_control import DirectMoveController
from Path.ackermann_odometry import AckermannOdometry

# 初始化死算里程计与控制器
odom = AckermannOdometry()
controller = DirectMoveController(odom, cmd_vel_pub_func=your_pub_cmd_vel_function)

# 1. 前进前往 (X=2.5m, Y=1.0m, Theta=90°)，速度 0.5m/s
controller.move_to(target_x=2.5, target_y=1.0, target_theta_deg=90.0, speed=0.5, mode="forward")

# 2. 倒车前往 (X=1.0m, Y=0.0m, Theta=0°)，速度 0.3m/s
controller.move_to(target_x=1.0, target_y=0.0, target_theta_deg=0.0, speed=0.3, mode="reverse")
```

### 您自定义的路线列表 `move_path`

```python
# 自己规划的航点路线列表: (x, y, theta_deg, speed, mode)
my_route = [
    (1.5, 0.0, 0.0,  0.5, "forward"),   # 前进到拐角 1
    (1.5, 3.5, 90.0, 0.4, "forward"),   # 前进到过道
    (2.5, 6.0, -90.0, 0.3, "reverse")   # 倒车接入 3号放货点
]

# 一键按顺序执行您的专属路线
controller.move_path(my_route)
```

