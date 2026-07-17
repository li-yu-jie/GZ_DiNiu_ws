# 🐂 地牛叉车 AMCL 自主导航任务交接文档

## 📋 任务概述
正在进行地牛叉车（Tricycle 单前轮驱动转向）的 AMCL 纯定位自主导航调试。最终目标是实现小车在已知地图下，能够流畅、平稳地自主导航，且当前货叉与前方障碍物发生碰撞前能安全避障停下。目前已完成了通信、底盘轮廓尺寸、避障雷达过滤和算法前瞻参数的联合调试，处于等待用户在 RViz 中定位对准并进行实车测试的阶段。

## 🧩 必要背景
* **实车物理尺寸**：长 1.85m, 宽 0.7m, 高 0.2m。
* **里程计零点（`base_link`）**：位于驱动轮中心。车尾在 $x = -0.25$m，货叉最前端在 $x = 1.60$m（$1.60 - (-0.25) = 1.85$m），小车物理宽度对应 $y \in [-0.35, 0.35]$。
* **激光雷达安装位置**：$x = 1.25$m。雷达发布的激光扫描话题 `/scan` 已经处于 `base_link` 坐标系。
* **控制方式**：AMCL 纯定位自主导航。使用 FAST-LIO 提供高精 $odom \to base\_link$ 坐标变换，AMCL 提供 $map \to odom$ 坐标变换。
* **约束与偏好**：必须对雷达数据进行自反射过滤，防止扫描到自身的金属货叉；避障必须高灵敏、不漏报；小车线速度分母奇异性会导致前轮左右剧烈摆动，必须通过调大预瞄距离等参数来平滑。

## ✅ 已完成的内容

### 1. laserscan_filter.py (雷达阴影及货叉剔除节点)
修正了雷达数据过滤范围，将过大的 $2.60$ 米过滤上限缩窄为真实的货叉尖端 $1.60$ 米，并将 `laser_x_offset` 设为 `0.0`，修正为以 `base_link` 为基准。同时引入了 `Best Effort` QoS 机制：
```python
        # Configure Best Effort QoS to match pointcloud_to_laserscan /scan topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
```

### 2. diuniu_nav.launch.py (导航启动文件)
修正了传入过滤器的参数，使过滤器仅针对真实的物理车身范围（长 1.85m，宽 0.70m）进行雷达自遮挡剔除：
```python
    laserscan_filter = Node(
        package='diuniu_base',
        executable='laserscan_filter',
        name='laserscan_filter',
        parameters=[{
            'x_min': -0.25,
            'x_max': 1.60,
            'y_min': -0.35,
            'y_max': 0.35,
            'laser_x_offset': 0.0,
            'laser_y_offset': 0.0
        }],
        output='screen'
    )
```

### 3. nav2_params.yaml (导航核心参数)
* **还原碰撞多边形轮廓 (footprint)** 匹配 1.85m 真实车长：
  ```yaml
  footprint: "[[1.60, 0.35], [1.60, -0.35], [-0.25, -0.35], [-0.25, 0.35]]"
  ```
* **优化 FollowPath 控制器 (RPP) 参数** 平抑前轮左右扭动：
  ```yaml
      desired_linear_vel: 0.4
      lookahead_dist: 1.8
      min_lookahead_dist: 1.0
      max_lookahead_dist: 2.5
      lookahead_time: 2.0
      approach_velocity_scaling_dist: 0.8
      regulated_linear_scaling_min_radius: 0.5
      cost_scaling_dist: 0.8
  ```

### 4. 运行环境彻底清理
通过强杀后台残余进程，解决了多组底盘节点 (`diuniu_base`)、发布器和 SLAM 同时运行，导致读写串口冲突以及 `/tf` 坐标系打架的致命死锁。目前进程树 100% 干净。

## 🚀 下一步行动

### 1. 立即要做的第一件事
在新对话开始后，**立刻执行实车对齐与测试**：
* 在物理主机 RViz 界面中，点击 **`2D Pose Estimate`**，将红色定位箭头顺着实车物理方向对齐。
* 如果在 RViz 的 `/scan_filtered` 雷达显示中看不到点云，必须在左侧展开项中找到 **`QoS` -> `Reliability`**，将其由 `Reliable` 切换为 **`Best Effort`** 才能正确亮起。
* 点击 **`2D Goal Pose`** 发送导航目标点。

### 2. 后续步骤
1. **安全停障验证**：站在正在行驶的叉车正前方，测试叉车是否能在货叉触碰到人体前，灵敏、平稳地完全停下。
2. **前轮打角观察**：检查在直线加速、转弯和避障减速过程中，前轮左右剧烈摆动（画蛇扭动）现象是否已彻底消失。
3. **窄道通过性验证**：发送窄道目标点，检验经过平滑后的局部规划路径是否依然能保证长车身顺利通过。

### 3. 注意事项
* **避免同名节点冲突**：在启动导航时，绝对不能在多个终端重复运行同一个 launch 文件，否则会产生严重的 TF 冲突（甚至导致串口失效、底盘断电报警）。
* **QoS 级别保持**：由于雷达话题发布是 Best Effort，过滤话题发布也是 Best Effort，所以任何展示及订阅它的地方（包括 RViz 及自定义接收节点）都必须采用 Best Effort 的接收 QoS，否则通信将被 DDS 协议静默丢弃。
