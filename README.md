# 地牛小车 Mid360 雷达自主导航与底盘驱动运行指南

本指南记录了地牛小车（Tricycle 前驱转向车型，轴距 $1.30\text{m}$，半宽 $0.35\text{m}$）基于 **模式 A（FAST-LIO 高精直连 SLAM 定位导航）**、**模式 A-2（FAST-LIO + AMCL 地图匹配重定位）** 与 **模式 B（AMCL 2D 纯定位导航）** 的实车部署步骤、手柄遥控配置及多终端运行指令流。

> 快速启动：已提供一键启动全部导航节点 launch 文件 `diuniu_nav_all.launch.py`，可直接替代分终端启动。

---

## 💡 核心配置与防坑调试优化

为了保证小车在狭窄通道正常定位、顺畅过弯与稳定行驶，我们实施了以下核心系统优化：

1. **统一系统时钟时间戳（`time_sync_en: false`）**：
   雷达驱动已在底层统一使用工控机系统时钟为点云打时间戳，在 FAST-LIO 中关闭了软件同步（`time_sync_en: false`），避免了启动瞬时通信延时导致的 1.17 秒虚假时间偏置（该偏置会导致 LIO 定位死锁、不发布 `/odom` 话题）。
2. **过滤地面噪点避障（`min_height: 0.15`）**：
   将点云切片高度下限修改为地面上 15 厘米（`min_height: 0.15`，上限 `max_height: 1.2`），排除了地板瓷砖反光等噪点被识别为障碍物，解决了小车因为“地面积满障碍”而反复原地触发 Recovery 且无法行驶的 bug。
3. **雷达自身阴影与前货叉三维空间过滤器（`laserscan_filter`）**：
   * **原理**：雷达安装位置为前轮向后 8.5cm（即 $x = 1.215\text{m}$），前货叉最前端达 $x = 1.60\text{m}$，车身总长 1.85m（车尾为 $x = -0.25\text{m}$）。雷达极易扫描到车身及货叉尖端，导致代价地图误报自车障碍物锁死小车。
   * **解决**：启动 `laserscan_filter` 过滤节点。自动剔除 `base_link` 坐标系下长 $x \in [-0.25, 1.60]$ 米、宽 $y \in [-0.35, 0.35]$ 米区域内的雷达点云，并将处理后的干净数据以 `/scan_filtered` 话题发布。
   * **代价地图订阅**：局部和全局代价地图已重新启用实时避障（`obstacle_layer`），并将扫描源切换为过滤后的安全话题 `/scan_filtered`。
4. **20Hz 串口看门狗（Watchdog）与到点防漂移**：
   底盘驱动 `diuniu_base_node` 引入了控制看门狗机制。当导航成功到点后，控制器停止发布 `/cmd_vel`。一旦超过 0.2 秒没有接收到任何话题控制，底盘看门狗自动介入，**持续高频向串口发送速度和转向均为 0 的锁死数据包**，彻底根治了到点后由于旧指令残留导致叉车继续缓慢爬行、越滑越远的硬件漂移问题。
5. **手柄与导航双通道隔离（原地防走车）**：
   * 手柄遥控节点 `diuniu_teleop_cmd_vel` 被重定向发布到专属的 `/cmd_vel_joy`。
   * 底盘驱动订阅双话题。当车辆静止时（$v=0$），**手柄转向只会控制前轮的原地偏转（对齐），驱动电机锁定为 0**，有效保护机械结构且杜绝了原地挪方向时意外走车的安全风险。而导航 `/cmd_vel` 原地自转功能（前轮 $90^\circ$ 旋转驱动）不受影响。
6. **阻尼前轮消除剧烈抖动与电机防热**：
   在 RPP 控制器中，将最小预瞄距离 `min_lookahead_dist` 上调至 **`0.9` 米**。这为转向回路提供了充足的控制阻尼，**彻底消除了低速转弯时前轮频繁剧烈扭摆、转向盘电机发热过烫的问题**，并保证了牵引轮动力输出连贯。
7. **全局大拐弯设计（解决车头过车尾卡）**：
   * 禁用 RPP 原地自转（`use_rotate_to_heading: false`），防止货叉原地 2.9 米旋转扫墙导致避障锁死。
   * 将 `global_costmap` 膨胀半径设为 **`0.55` 米**，并将全局代价衰减因子调小至 **`1.5`**。这会强迫全局规划路径走在通道和门洞的“正中间”，并在拐弯处主动靠外侧弯（大拐弯），为车尾的内切（内轮差）留出 10cm 的富余车宽，车尾绝不再刮内墙角。
   * `local_costmap` 膨胀半径设为 **`0.45` 米**，配合 2.0 衰减因子，安全防刮。

8. **坐标系修正（base_link 与 laser_link 对齐）**：
   * `base_link` 定义在两后轮中心连线中点（车辆运动学参考点）。
   * 激光雷达安装在 `laser_link`，位于 `base_link` 前方 $1.215\text{m}$、上方 $0.6\text{m}$ 处。
   * FAST-LIO 内部估计的是 IMU/LiDAR 帧位姿，原始代码直接把它发布为 `base_link`，导致车身模型在 RViz 中整体向前偏移约 $1.2\text{m}$。
   * 已在 `src/FAST_LIO/src/laserMapping.cpp` 中修正：`/Odometry`、`odom → base_link` TF、`/cloud_registered_body` 点云均在发布前转换到真正的 `base_link` 系。

9. **禁用 Spin 原地旋转恢复（根治“导航中突然转 90°、持续转圈撞墙不停”事故）**：
   * **现象**：导航途中车辆突然原地旋转 90°，随后反复转圈，撞到墙也不停止。
   * **根因**：Nav2 默认行为树中卡死恢复动作包含 `Spin spin_dist="1.57"`（原地转 90°）。Tricycle 前驱转向三轮车**无法真正原地自转**，底盘只能以“前轮打 ±90° + 高速驱动”近似旋转，车身实际扫掠面积远大于 Nav2 Spin 插件按 footprint 纯旋转仿真的碰撞检测范围——仿真认为安全，实车却扫墙/撞墙；且 `RecoveryNode` 会多次重试（清代价地图 → Spin → Wait → BackUp 循环），表现为持续转圈撞墙不停。
   * **修复**：
     * 新增自定义行为树 `src/diuniu_nav/behavior_trees/navigate_to_pose_w_replanning_and_recovery_no_spin.xml`，恢复动作仅保留 **清代价地图 → Wait 5s → BackUp 0.30m**；`diuniu_nav.launch.py` 启动时通过 `RewrittenYaml` 自动注入其绝对路径，无需手动配置。
     * `behavior_server` 的 `behavior_plugins` 移除 `spin`，仅保留 `backup`/`wait`。
     * `progress_checker` 卡死判定时限由 3s 放宽至 **6s**（`movement_time_allowance: 6.0`），减少长车身重载低速蠕行时的误触发。
   * **注意**：恢复行为（behavior_server）直接发布 `/cmd_vel`，不经过 velocity_smoother，动作较突兀；BackUp 为低速后退（0.05 m/s），后方无雷达视野，请在相对开阔区域使用。

---

## 🚀 一键启动全部导航节点（推荐）

如果觉得分终端启动麻烦，可以使用新增的一键启动文件，它会在一个终端内顺序启动雷达、FAST-LIO、底盘、Nav2：

```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav_all.launch.py
```

* 默认启用 **模式一 B**（`use_amcl:=false use_relocalization:=true`），即 FAST-LIO + AMCL 地图匹配重定位。
* 如需使用模式一 A（静态原点），可传参：
  ```bash
  ros2 launch diuniu_nav diuniu_nav_all.launch.py use_relocalization:=false
  ```

> ⚠️ 一键启动会占用一个终端长期输出日志；后台运行请使用 `screen`/`tmux`，或自行 `nohup` 重定向。

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

### 2. 启动 SLAM 里程计（用于提供高精 `odom` -> `base_link` 位姿）
在 **【终端 2】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py rviz:=false
```
* **验证**：输出 `IMU Initial Done`，提供高精三维惯导里程计，并且 `/Odometry` 话题满速发布。

### 3. 启动小车底盘节点（关闭底盘自身的里程计，防冲突）
在 **【终端 3】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_base diuniu_base.launch.py pub_odom_tf:=false pub_odom_topic:=false
```
* **验证**：开始周期输出 `🔍 [底盘发包] v_front=0.000 m/s, alpha=0.00°, lift=0 | dt=0.03s`，看门狗就绪。

---

## 🧭 选择导航或手柄遥控运行模式

### 📍 模式一：直连 SLAM 自主导航（FAST-LIO 里程计 + 可选 AMCL 地图匹配重定位）

模式一分为两种子模式，按需选择：

#### 模式一 A：纯 FAST-LIO 静态原点（★ 终极推荐，开机即准，防漂移能力极强）
如果您每次都能从**建图原点**启动车辆，想获得厘米级、无初始化的极致定位：
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false
```
* **特点**：不需要手动使用 `2D Pose Estimate` 对准，开机直接定位，运行中绝对不飘。
* **前提**：车辆启动时的物理位置和航向必须与建图原点一致（因为 launch 里把 `map → odom` 发布为固定单位变换）。

#### 模式一 B：FAST-LIO + AMCL 地图匹配重定位（开机位置任意，需一次初始对齐）
如果您希望开机位置**不必回到建图原点**，让 AMCL 用过滤后的 `/scan_filtered` 与 2D 栅格地图匹配，动态算出 `map → odom`：
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=false use_relocalization:=true
```
* **特点**：FAST-LIO 继续提供高精 `odom → base_link` 里程计，AMCL 负责地图匹配并发布 `map → odom`，两者优势互补。
* **初始位姿**：首次启动后，请在 RViz 中使用 **`2D Pose Estimate`** 给出车辆的粗略位置和朝向；随后 AMCL 会自动跟踪。如果车辆恰好从建图原点附近启动，也可省略这一步。

### 📍 模式二：AMCL 2D 定位导航（支持任意位置手动定位对齐）
如果您需要使用传统的静态 2D 栅格地图匹配：
在 **【终端 4】** 中运行：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_nav diuniu_nav.launch.py use_amcl:=true
```
* **特点**：支持全局代价地图 120 束激光的高精度匹配，适合常规工况。

### 📍 模式三：话题模式手柄遥控（无冲突，无锁死）
在需要手柄介入遥控时，在 **【终端 4】** 中运行：
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

## 🖥️ 启动可视化监控与操作指南

在 **【终端 5】** 中运行（需在 NoMachine 图形桌面环境内）：
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 1. 手动重定位对齐（针对模式二 AMCL 与模式一 B）
* 点击 RViz 界面顶部的 **`2D Pose Estimate`**（红色定位箭头图标）。
* **关键**：点击位置必须是车辆的 **`base_link`**，即**两后轮中心连线中点**（车尾方向、车身中心线上），而不是车头或雷达位置。
* 在地图上**点击并按住左键，顺着小车车头的实际物理方向拉出红色箭头，松开**。
* 控制小车**前后移动半米**，AMCL 定位会瞬间“吸附”对齐，红色的雷达点云与 2D 地图墙壁完美重合，彩色代价地图完全亮起。
* 如果点错位置（例如点在车头），车身模型会整体偏移约 $1.2\text{m}$，出现过门时“车尾已过、实际才到车头”的错觉。

### 2. 观察过滤后的雷达图层（Laserscan Filter）
* 在 RViz 左侧的 `Displays` 列表中，展开 **`LaserScan`** 属性。
* 将订阅的话题（Topic）修改为 `/scan_filtered`，可以看到车身和货叉部分的反射点已经被 100% 滤除干净，只有车外障碍物被正常保留。

### 3. 加载 URDF 3D 车辆模型（若模型没有显示）
* 在 RViz 左侧的 `Displays` 列表中，展开 **`RobotModel`** 左侧小三角。
* 确保 `Description Source` 设置为 **`Topic`**。
* 展开 `Description Topic` 下的 **`QoS`**，将 **`Durability`** 从 `Volatile` 修改为 **`Transient Local`**。
* 3D 模型便会瞬间正确绘制在原点。

### 4. 开始自主导航
* 点击 RViz 窗口顶部的 **`2D Goal Pose`** 按钮。
* 选择距离小车较远（建议 2 米开外）的空白地带，**按住左键拖拽方向并松开**。
* 局部寻迹器（RPP）会规划大拐弯动线，地牛小车稳定且安全地平稳行驶至目标点！
