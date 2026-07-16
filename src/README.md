# 地牛小车 Mid360 雷达建图与地图保存完整指南

本指南详细记录了从启动激光雷达驱动，到运行 FAST-LIO2 建图，再到使用 `pcd2pgm` 转换为 2D 栅格地图的全部指令流和操作步骤。

---

## 🛠️ 第一阶段：准备工作（IP 与网络确认）

雷达 IP 静态配置为 `172.21.22.22`，小车接收端 IP 为 `172.21.22.21`。每次小车开机或有线网口重新连接时，请确认网卡 IP 状态。

1. **检查网络连通性**：
   ```bash
   ping 172.21.22.22 -c 3
   ```
   * 确保能正常收到雷达的响应。

---

## 🚀 第二阶段：启动建图程序

请在小车的主机上打开 **3 个终端**（SSH 或 NoMachine 终端均可），分别执行以下步骤：

### 1. 启动雷达驱动
在 **【终端 1】** 中运行：
```bash
source ~/GZ_DiNiu_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
* **验证**：看到输出 `Init lds lidar success!` 和 `GetFreeIndex key:livox_lidar_...` 说明雷达成功向外广播点云。

### 2. 启动 FAST-LIO2 SLAM 算法
在 **【终端 2】** 中运行：
```bash
source ~/GZ_DiNiu_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py rviz:=false
```
* **参数说明**：`rviz:=false` 代表关闭小车本机的图形化 RViz，防止消耗显卡/CPU，仅在后台进行 SLAM 位姿计算。
* **验证**：看到输出 `Node init finished.` 和 `Multi thread started` 说明 SLAM 正常接收数据并开始计算。

### 3. 启动 RViz2 可视化监视
在 **【终端 3（必须是 NoMachine 图形界面终端）】** 中运行：
```bash
source ~/GZ_DiNiu_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/GZ_DiNiu_ws/src/FAST_LIO/rviz_cfg/fastlio.rviz
```

#### 💡 RViz2 话题配置步骤：
* **对齐坐标系**：将左侧面板最上方的 **`Fixed Frame`** 从 `map` 改为 **`odom`**，此时 `Global Status` 会变为绿色的 `Ok`。
* **添加点云**：点击左下角 **`Add`**，选择 **`PointCloud2`**。展开属性将 **`Topic`** 设为 **`/cloud_registered`**（SLAM 注册后的实时高精点云）或 **`/Laser_map`**（全局拼接的地图）。
* **添加轨迹**：点击左下角 **`Add`**，选择 **`Path`**。展开属性将 **`Topic`** 设为 **`/path`**，可观察小车轨迹。

---

## 💾 第三阶段：手柄扫图与主动保存 3D 地图

1. **手柄控制小车建图**：
   长按手柄使能键，控制小车在整个场地内**慢速匀速绕行一圈**。
   * *⚠️ 注意：在转弯和进出窄道时务必控制车速极慢，以保证点云拼接质量。*

2. **服务调用，主动保存 3D 点云**：
   建图结束后，不要强行 Ctrl+C。请在小车上**新开一个终端**，调用 ROS 2 服务：
   ```bash
   ros2 service call /map_save std_srvs/srv/Trigger {}
   ```
   * **验证**：终端返回 `success=True, message="Map saved."`，3D 点云地图已安全写入 `/home/y/GZ_DiNiu_ws/src/FAST_LIO/PCD/scans.pcd`。
   * 保存成功后，您可以回到运行 `fast_lio` 的**【终端 2】**中按下 **`Ctrl + C`** 退出 SLAM。

---

## 🗺️ 第四阶段：转换为 2D 导航地图（PGM & YAML）

1. **启动 pcd2pgm 转换节点**：
   在小车终端运行：
   ```bash
   source ~/GZ_DiNiu_ws/install/setup.bash
   ros2 launch pcd2pgm pcd2pgm_launch.py
   ```
   * **作用**：该节点会自动加载 `scans.pcd`，经过高度截取和滤波后发布成标准的 `/map` 栅格话题。

2. **保存 2D 栅格地图文件**：
   新开一个终端，执行以下命令保存并落盘：
   ```bash
   # 创建地图保存文件夹
   mkdir -p ~/GZ_DiNiu_ws/src/maps/
   
   # 保存地图话题为 map.pgm 和 map.yaml
   ros2 run nav2_map_server map_saver_cli -f ~/GZ_DiNiu_ws/src/maps/map
   ```

3. **关闭转换节点**：
   保存完成后，在运行 `pcd2pgm` 的终端中按下 **`Ctrl + C`** 关闭该节点。

---

## 📂 最终成果

执行完毕后，您将在 **`/home/y/GZ_DiNiu_ws/maps/`** 目录下获得两个文件：
* **`map.pgm`**：2D 栅格灰度地图图像。
* **`map.yaml`**：地图元配置文件，包含地图分辨率、起点坐标、占用阈值等。

这两个文件将作为后续 **Nav2 自主导航** 的输入源底图！
