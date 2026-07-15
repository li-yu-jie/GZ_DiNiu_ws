# diuniu_description 可视化模型包说明文档

> **功能描述**：本功能包包含了地牛（DiuNiu）机器人的 1:1 三维物理描述模型（URDF）、可视化启动脚本（Launch）以及预设的 RViz2 图形界面配置文件。

---

## 1. 物理模型规格 (diuniu.urdf)

模型以 **两后轮连线中心** 为核心原点（`base_link`），物理参数均经过精确测绘：

| 部件名称 | 物理形状与尺寸 | 在 `base_link` 下的安装位置 (xyz) | 材质与外观颜色 |
| :--- | :--- | :--- | :--- |
| **车架主箱体** | 长方体: $1.85\text{m} \times 0.7\text{m} \times 0.2\text{m}$ | `0.675 0 0.18` (底盘抬高以留出 8cm 离地间隙) | 工业橙色 (`orange`, 40% 半透明度) |
| **前驱动转向轮** | 圆柱体: 半径 $0.085\text{m}$ (直径17cm)，厚 $0.08\text{m}$ | `1.3 0 0.085` (正前方 1.3 米轴距处) | 黑色 (`black`, 不透明) |
| **左后从动轮** | 圆柱体: 半径 $0.085\text{m}$ (直径17cm)，厚 $0.05\text{m}$ | `0 0.265 0.085` (左侧 26.5cm 轮距处) | 黑色 (`black`, 不透明) |
| **右后从动轮** | 圆柱体: 半径 $0.085\text{m}$ (直径17cm)，厚 $0.05\text{m}$ | `0 -0.265 0.085` (右侧 26.5cm 轮距处) | 黑色 (`black`, 不透明) |
| **IMU 传感器** | 长方体: $0.05\text{m} \times 0.05\text{m} \times 0.02\text{m}$ | `0.675 0 0.2` (位于底盘正中心上方) | 蓝色 (`blue`, 不透明) |
| **激光雷达 (Lidar)** | 圆柱体: 半径 $0.035\text{m}$，高 $0.05\text{m}$ | `1.25 0 0.6` (车头偏后 35cm 支架高度 0.6m) | 红色 (`red`, 不透明) |

> [!TIP]
> **设计亮点**：
> - **半透明橙色车身**：让您在仿真中无需隐藏车体，即可透视看到内部蓝色的 **`imu_link`** 坐标状态。
> - **真实离地间隙**：车身整体抬高 `0.18m`，让底盘轮子以真实的物理质感在车底外露滚动。

---

## 2. 文件结构

```text
diuniu_description/
├── CMakeLists.txt
├── package.xml
├── setup.py                    # 配置 launch, urdf, rviz 的资源拷贝
├── README.md                   # 本说明文档
├── urdf/
│   └── diuniu.urdf             # 精确 1:1 地牛机器人 URDF 描述文件
├── launch/
│   └── display.launch.py       # 启动节点并全自动调起 RViz2 布局
└── rviz/
    └── diuniu.rviz             # 预设的 RViz2 场景配置文件
```

---

## 3. 快速启动与部署

请在您的小主机终端执行以下命令：

### 第一步：编译功能包
```bash
cd ~/GZ_DiNiu_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select diuniu_description
```

### 第二步：加载环境变量并启动
```bash
source install/setup.bash
ros2 launch diuniu_description display.launch.py
```

> [!NOTE]
> **自适应免配置机制**：
> 启动后，RViz2 将自动读取 `diuniu.rviz` 并完成以下配置：
> 1. 将 **Fixed Frame** 锁定为 **`odom`**（开启“上帝视角”以支持轨迹展示，车体会随运动在网格上移动）。
> 2. 自动载入机器人 3D 骨架，并高亮显示 TF 坐标树。
> 3. 自动添加 **Odometry 轨迹展示**，默认订阅 `/odom` 话题，并实时保留最近 **100 步**的运动红色路径。

---

## 4. 联合实车运行测试

当您需要虚拟模型在网格上实时同步实体车辆的行驶与转向动作时，请依次打开三个终端运行：

### 1️⃣ 终端 1：拉起底盘物理通信节点
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_base diuniu_base.launch.py
```

### 2️⃣ 终端 2：启动可视化与状态发布树
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch diuniu_description display.launch.py
```

### 3️⃣ 终端 3：拉起手柄遥控节点
```bash
cd ~/GZ_DiNiu_ws
source install/setup.bash
ros2 launch betop_teleop diuniu_teleop.launch.py
```

**控制效果**：按住手柄 **LB** 键并推动摇杆，实体地牛在地面行驶的同时，RViz2 里的半透明橙色 3D 机器人模型会在虚拟网格上**完全同步前进、后退、转弯与原地打转**，且会自动生成一条红色的运动轨迹线！

---

## 5. RViz2 中手动配置历史轨迹步骤（参考）

如果您未来由于新建场景或修改布局需要手动配置轨迹显示，可参考以下步骤：

### 第一步：切换为“上帝视角”（关键）
- 在 RViz2 左侧的 **Displays** 面板最上方，找到 **Global Options**。
- 将 **Fixed Frame** 从 `base_link` 改为 **`odom`**。
  *(注：`base_link` 是第一人称视角，车体会永远锁定在网格中心不动；只有改为 `odom`，车体才会随着实车坐标在网格上开走。)*

### 第二步：添加 Odometry 显示项
- 点击 Displays 面板左下角的 **Add** 按钮。
- 在弹出的组件列表窗口里向下滚动，找到并双击 **`Odometry`** 进行添加。

### 第三步：配置话题与轨迹参数
- 在 Displays 面板中展开新添加的 `Odometry`。
- 找到 **Topic** 属性，点击其右侧，选择或手动输入 **`/odom`**。
- **配置历史步数（保留长线）**：向下滚动找到 **`Keep`** 属性（默认是 1），将其修改为 **`100`** 或 **`500`**。
- **自定义外观（可选）**：
  - 找到 **`Shape`** 属性，可将 `Arrow`（箭头）更改为 `Line`（线条）或 `Point`（点）。
  - 找到 **`Color`** 属性，可将其修改为亮红色或其他易于辨识的颜色。
- **保存配置**：
  - 配置完成后，按 **`Ctrl + S`** 将此配置保存覆盖到 `rviz/diuniu.rviz` 中，以便下次启动时直接一键加载。

