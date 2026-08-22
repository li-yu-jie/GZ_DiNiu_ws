# diuniu_apriltag — 相机标定与 AprilTag 识别

DiuNiu 机器人视觉感知包：XW500U3 USB 相机驱动 → 图像去畸变 → AprilTag(36h11) 位姿识别。

> 现役相机：**XW500U3**（1920x1080@30fps，仅 MJPG → 必须用 usb_cam 驱动）。
> 历史相机：Orbbec DaBai DCW（640x480，v4l2_camera，已退役）、AF093V1（未标定即换下）。

## 数据流

```
usb_cam (相机驱动, 1920x1080 MJPG, 加载标定内参)
    ├─ /image_raw
    └─ /camera_info_raw ──→ camera_info_relay (补 frame_id)
                                └─ /camera_info
/image_raw + /camera_info ──→ image_proc/rectify_node (去畸变)
                                └─ /image_rect
/image_rect + /camera_info ──→ apriltag_node (apriltag_ros)
                                ├─ /detections   (检测结果数组)
                                └─ /tf: camera_optical_frame → tag0  (3D 位姿)
```

> **为什么需要 camera_info_relay**：usb_cam 的 `/camera_info` frame_id 需要统一为
> `camera_optical_frame`，否则 apriltag_ros 无法发布 TF。中继节点补上 frame_id 后转发。

## 快速启动（推荐）

```bash
# 宿主机上一键启动（自动定位相机设备号，容器内拉起全链路）
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh

# 带 RViz 可视化（相机画面 + tag 坐标轴叠加）
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh rviz:=true
```

- `start_apriltag.sh` 先调用容器内 `ensure_camera.sh`：按名字 `XW500U3` 扫描
  sysfs 找到当前设备号，容器 /dev 缺节点则自动 `mknod` 补齐（容器 /dev 是启动
  时快照，相机重插后必须这一步），再把设备路径传给 launch。
- 可选：宿主机装 udev 规则固定软链接 `/dev/xw500u3`（需 sudo，见
  `scripts/99-xw500u3.rules` 头部注释）。**注意该规则对容器无效**，容器内
  永远以 `ensure_camera.sh` 为准。

实时距离监视（宿主机执行，弹出 "AprilTag 实时距离" 窗口，10Hz 刷新 z/x/y/偏航）：

```bash
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_distance_watch.sh
```

## 一、相机内参标定（已完成，仅重标时需要）

使用**官方** `camera_calibration` 包。

### 标定板

| 参数 | 值 |
|---|---|
| 方块阵列 | 6 行 × 9 列 |
| 方块边长 | 实测 20 mm |
| OpenCV 内角点 | **8 列 × 5 行**（方块数各减 1） |

### 标定步骤

```bash
# 1. 安装（容器内，只需一次）
sudo apt install ros-humble-camera-calibration

# 2. 先起相机（用本包的 launch 即可）
~/GZ_DiNiu_ws/src/diuniu_apriltag/scripts/start_apriltag.sh

# 3. 另开终端，启动标定 GUI
ros2 run camera_calibration cameracalibrator --size 8x5 --square 0.020 \
    --ros-args -r image:=/image_raw
```

4. 手持标定板在视野内移动/旋转/远近变化，直到 X / Y / Size / Skew 四个进度条变绿
5. **CALIBRATE** → 看重投影误差（XW500U3 镜头偏软，**RMS ~1 px 即合格**）
6. **SAVE** → 数据存到 `/tmp/calibrationdata.tar.gz`（内含样本图和 `ost.yaml`）
7. **COMMIT** 对 usb_cam 无效，直接部署 `ost.yaml`：

```bash
# 部署位置：功能包内（launch 默认加载这里），改完需重新编译 diuniu_description
src/diuniu_description/config/camera/xw500u3_1920x1080.yaml

# usb_cam 也会读 ~/.ros/camera_info/<camera_name>.yaml 作为后备
```

### 现役标定结果（XW500U3，2026-08-22，1920x1080）

```
fx=1721.32  fy=1743.82  cx=952.27  cy=608.64
畸变 (plumb_bob): k1=-0.0539  k2=0.1217  p1=0.0001  p2=-0.0032  k3=0
```

内参文件：`src/diuniu_description/config/camera/xw500u3_1920x1080.yaml`
样本备份：`src/diuniu_description/config/camera/calibrationdata_xw500u3_20260822.tar.gz`

**标定教训：**
- 采样时**保持自动曝光/自动增益**——手动长曝光(60ms)+手抖 = 全批运动模糊废掉
- 每位置停 2 秒再换；棋盘占画面 1/3~1/2，距离 40~70cm
- XW500U3 的 `focus_absolute` 控制疑似无效（扫描清晰度曲线平坦），别折腾对焦
- SAVE 没反应先看日志：`/tmp/calibrationdata.tar.gz` 若被 root 占用会静默失败

历史：Orbbec 640x480 标定（fx=623.15，RMS 0.23px）见 `orbbec_rgb_640x480.yaml`。

## 二、AprilTag 识别

### 现场标签

- 家族：**36h11**（6×6 数据位 + 1 格黑边框 = 8×8 格），ID = **0**
- 有效边长：**94.1 mm**（黑色外缘）——2026-08-22 三点卷尺现场标定值，见下节
- 打印在纸上

### 标签边长与距离尺度（重要）

**距离精度原理**：`z = fx × 标签边长 / 像素边长`。fx 由内参标定给出，边长是
唯一的外部尺度来源——边长错多少比例，距离就错多少比例，且**误差随距离放大**。

2026-08-22 现场标定（症状："距离越大误差越大"，70cm 处偏小近 10cm）：

| 卷尺真值（0 点顶镜头口） | 修正前读出 |
|---|---|
| 20 cm | 17.8 cm |
| 50 cm | 43.5 cm |
| 70 cm | 60.2 cm |

三点拟合 `z_read = 0.850 × z_true`，残差 ≤1.6mm → 有效边长 = 80/0.850 =
**94.1mm**。隐含 fx 反推与内参标定值 1721 吻合，证明黑色外缘实为 ~94mm
（此前"80mm"系误测）。修正写入 `config/apriltag.yaml` 的 `tag.sizes` 后：

- 70cm 处读出 **70.8cm**（修正前 60.2cm）
- 误差不再随距离放大；剩**恒定 +1cm**（光学中心在镜头口前方 ~1cm，
  装车后由相机→base_link 外参自然吸收，手持验证阶段忽略）

**若更换标签/重打印**：量黑色外缘边长填入 `tag.sizes`；若距离仍有随距离放大
的偏差，重复上面的三点标定（摆 20/50/70cm 三档记录读出，拟合比例 k，
`tag.sizes /= k`）。

### 启动（手动方式）

```bash
# 编译（源码改动后）
cd ~/GZ_DiNiu_ws && colcon build --packages-select diuniu_apriltag
source install/setup.bash

# 启动整条链路（设备号以 ensure_camera.sh / ls /dev/video* 实际为准）
ros2 launch diuniu_apriltag apriltag.launch.py video_device:=/dev/video3

# 带 RViz 可视化
ros2 launch diuniu_apriltag apriltag.launch.py video_device:=/dev/video3 rviz:=true

# 临时改标签尺寸（只影响默认 size；tag0 的 TF 尺度以 config/apriltag.yaml 的 tag.sizes 为准）
ros2 launch diuniu_apriltag apriltag.launch.py tag_size:=0.0941
```

> **改了 apriltag.yaml 必须重启 launch**：`tag.sizes` 等参数是只读的，
> 不支持 `ros2 param set` 热更新。

### 验证数据流

```bash
# 1. 检测结果（有码时应持续输出，hamming=0、decision_margin 越大越可靠）
ros2 topic echo /detections

# 2. 3D 位姿（距离看 Translation 的 z，单位米）
ros2 run tf2_ros tf2_echo camera_optical_frame tag0

# 3. 图像健康检查
ros2 topic hz /image_raw      # 1080p MJPG 下 ~15-26 Hz 波动正常

# 4. 确认标签尺寸参数已生效
ros2 param get /apriltag tag.sizes   # 应为 [0.0941]
```

### 关键配置 [config/apriltag.yaml](config/apriltag.yaml)

| 参数 | 当前值 | 说明 |
|---|---|---|
| `family` | `36h11` | 标签家族 |
| `size` / `tag.sizes` | `0.0941` | 黑色方块外缘边长（米），**距离精度的唯一尺度来源** |
| `max_hamming` | `0` | 只接受零误码检测，最严格 |
| `detector.decimate` | `1.0` | 不降采样（保精度） |
| `tag.ids/frames/sizes` | `[0]/[tag0]/[0.0941]` | **只有这里配置的标签才会发 TF** |

## 三、常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `Failed opening device: Permission denied` | 用户不在 video 组：`sudo usermod -aG video y`，重新登录 |
| usb_cam 崩 `Invalid v4l2 format` | USB 链路不稳定导致设备瞬间掉线重枚举；重启 launch。严重时就拔插 USB 并重跑 `start_apriltag.sh` |
| v4l2_camera 崩 `Current pixel format is not supported yet: MJPG` | XW500U3 只有 MJPG，v4l2_camera 解不了——这就是换 usb_cam 的原因，别换回去 |
| `Device or resource busy` | 有残留相机进程，kill 掉再起（旧 launch 没退干净时重开就会这样） |
| 重插 USB 后容器里找不到相机 | 容器 /dev 是启动时快照：重跑 `start_apriltag.sh`（内部 ensure_camera.sh 会自动 mknod 补节点） |
| `/tf` 里没有 tag0 | ① 码没被识别（看 `/detections`）；② `tag.ids` 没配置该 ID |
| TF 报 `camera_optical_frame does not exist` | camera_info_relay 没起来，或 frame_id 不一致 |
| `/camera_info` 的 K 全是 0 | 标定 yaml 没被安装/加载：检查 `diuniu_description` 的 setup.py 有 config/camera 安装规则并重新 colcon build |
| `/detections` 的 frame_id 是 `default_cam` | usb_cam 的坐标系参数名是 **`frame_id`** 不是 `camera_frame_id`，写错会被静默忽略 |
| RViz 里 Camera 显示红点 | 数据流一般没问题，先取消再勾上 Camera 的勾选框强制重订阅 |
| 改了 apriltag.yaml 不生效 | 参数只读，必须重启 launch；用 `ros2 param get /apriltag tag.sizes` 验证 |
| 标定 GUI 黑窗 | 把窗口顶部 `scale` 滑块往右拖即可显示 |

## 四、文件清单

```
diuniu_apriltag/
├── launch/apriltag.launch.py        # 一键启动 5 节点（相机/中继/去畸变/识别/RViz）
├── config/apriltag.yaml             # 检测器 + 标签尺寸配置（距离尺度权威来源）
├── diuniu_apriltag/camera_info_relay.py  # camera_info 补 frame_id 中继
├── scripts/start_apriltag.sh        # 宿主机一键启动识别链路（自动定位相机）
├── scripts/start_distance_watch.sh  # 宿主机弹出实时距离窗口
├── scripts/ensure_camera.sh         # 容器内定位 XW500U3 并 mknod 补 /dev 节点
├── scripts/distance_watch.py        # 实时距离监视（被 start_distance_watch.sh 调用）
├── scripts/99-xw500u3.rules         # 宿主机 udev 固定 /dev/xw500u3（可选，需 sudo）
├── rviz/apriltag.rviz               # RViz 布局：相机画面 + TF 坐标轴
└── README.md
```

内参文件在 `diuniu_description/config/camera/xw500u3_1920x1080.yaml`（历史：
`orbbec_rgb_640x480.yaml`）。

## 待办

- [ ] **相机→base_link 外参标定**：需实测相机在车上的安装位姿（x/y/z/俯仰），
      加静态 TF 到 diuniu_description。当前 +1cm 恒定偏移（光学中心在镜头口
      前方）也由此外参一并吸收。
