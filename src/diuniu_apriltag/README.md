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

# 2. 先起相机（只需相机节点，可直接用本包的 launch）
ros2 launch diuniu_apriltag apriltag.launch.py video_device:=/dev/video3

# 3. 另开终端，启动标定 GUI
ros2 run camera_calibration cameracalibrator --size 8x5 --square 0.020 \
    --ros-args -r image:=/image_raw
```

4. 手持标定板在视野内移动/旋转/远近变化，直到 X / Y / Size / Skew 四个进度条变绿
5. **CALIBRATE** → 看重投影误差（本次结果 **RMS 0.23 px**，< 0.5 即合格）
6. **SAVE** → 数据存到 `/tmp/calibrationdata.tar.gz`（内含样本图和 `ost.yaml`）
7. **COMMIT** 对 v4l2_camera 无效（无 `set_camera_info` 服务），直接把 `ost.yaml`
   改名部署：

```bash
# 部署位置 1：功能包内（launch 默认加载这里）
src/diuniu_description/config/camera/orbbec_rgb_640x480.yaml

# 部署位置 2：v4l2_camera 默认搜索路径（camera_name 须与驱动上报一致）
~/.ros/camera_info/orbbec_dabai_dcw_rgb_camera:_or.yaml
```

改完功能包内的 yaml 后需重新编译 `diuniu_description` 才生效。

### 现役标定结果（XW500U3，2026-08-22，1920x1080）

```
fx=1721.32  fy=1743.82  cx=952.27  cy=608.64
畸变 (plumb_bob): k1=-0.0539  k2=0.1217  p1=0.0001  p2=-0.0032  k3=0
```

内参文件：`src/diuniu_description/config/camera/xw500u3_1920x1080.yaml`
样本备份：`src/diuniu_description/config/camera/calibrationdata_xw500u3_20260822.tar.gz`

**标定教训（XW500U3 镜头偏软，RMS ~1px 即合格）：**
- 采样时**保持自动曝光**——手动长曝光(60ms)+手抖 = 全批运动模糊废掉
- 每位置停 2 秒再换；棋盘占画面 1/3~1/2，距离 40~70cm

历史：Orbbec 640x480 标定（fx=623.15，RMS 0.23px）见 `orbbec_rgb_640x480.yaml`。

## 二、AprilTag 识别

### 现场标签

- 家族：**36h11**（6×6 数据位 + 1 格黑边框 = 8×8 格），ID = **0**
- 尺寸：**黑色方块外缘 80 mm**（每格 10 mm，已实测）
- 打印在纸上

### 启动

```bash
# 编译（源码改动后）
cd ~/GZ_DiNiu_ws && colcon build --packages-select diuniu_apriltag
source install/setup.bash

# 启动整条链路（设备号以 ls /dev/video* 实际为准，Orbbec RGB 那个）
ros2 launch diuniu_apriltag apriltag.launch.py video_device:=/dev/video3

# 带 RViz 可视化（相机画面 + tag 坐标轴叠加）
ros2 launch diuniu_apriltag apriltag.launch.py video_device:=/dev/video3 rviz:=true

# 临时改标签尺寸（只影响默认 size；tag0 的 TF 尺度以 config/apriltag.yaml 的 tag.sizes 为准）
ros2 launch diuniu_apriltag apriltag.launch.py tag_size:=0.08
```

### 验证数据流

```bash
# 1. 检测结果（有码时应持续输出，hamming=0、decision_margin 越大越可靠）
ros2 topic echo /detections

# 2. 3D 位姿（距离看 Translation 的 z，单位米）
ros2 run tf2_ros tf2_echo camera_optical_frame tag0

# 3. 图像健康检查
ros2 topic hz /image_raw      # 应 ~15/30 Hz
```

### 关键配置 [config/apriltag.yaml](config/apriltag.yaml)

| 参数 | 当前值 | 说明 |
|---|---|---|
| `family` | `36h11` | 标签家族 |
| `size` / `tag.sizes` | `0.08` | 黑色方块外缘边长（米），**距离精度的唯一尺度来源** |
| `max_hamming` | `0` | 只接受零误码检测，最严格 |
| `detector.decimate` | `1.0` | 不降采样（小图保精度） |
| `tag.ids/frames/sizes` | `[0]/[tag0]/[0.08]` | **只有这里配置的标签才会发 TF** |

**距离精度原理**：`z = fx × 标签边长 / 像素边长`。
距离不准时只需核对两件事：内参 `fx`（标定给出）和 `tag.sizes`（尺子量黑色外缘）。

## 三、常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `Failed opening device: Permission denied` | 用户不在 video 组：`sudo usermod -aG video y`，重新登录 |
| usb_cam 崩 `Invalid v4l2 format` | USB 链路不稳定导致设备瞬间掉线重枚举；重启 launch。严重时就拔插 USB 并核对设备号 |
| `Device or resource busy` | 有残留相机进程，kill 掉再起 |
| 设备号变了 | USB 重插重新编号，启动前 `cat /sys/class/video4linux/video*/name` 确认 XW500U3 是哪个 |
| `/tf` 里没有 tag0 | ① 码没被识别（看 `/detections`）；② `tag.ids` 没配置该 ID |
| TF 报 `camera_optical_frame does not exist` | camera_info_relay 没起来，或 frame_id 不一致 |
| `/camera_info` 的 K 全是 0 | 标定 yaml 没被安装/加载：检查 `diuniu_description` 的 setup.py 有 config/camera 安装规则并重新 colcon build |
| 标定 GUI 黑窗 | 把窗口顶部 `scale` 滑块往右拖即可显示 |

## 四、文件清单

```
diuniu_apriltag/
├── launch/apriltag.launch.py        # 一键启动 5 节点（相机/中继/去畸变/识别/RViz）
├── config/apriltag.yaml             # 检测器 + 标签尺寸配置（距离尺度权威来源）
├── diuniu_apriltag/camera_info_relay.py  # camera_info 补 frame_id 中继
├── rviz/apriltag.rviz               # RViz 布局：相机画面 + TF 坐标轴
└── README.md
```

内参文件在 `diuniu_description/config/camera/orbbec_rgb_640x480.yaml`。

## 距离精度验证（2026-08-22，已通过）

三距离实测（30/35/50cm）：
- 像素几何反推与 TF 位姿完全自洽（如 50cm 处：0.409 vs 0.411 m）
- 距离比值正确（z50/z30 = 1.868，理论 1.909，差在手持放置）
- 绝对精度 ~1cm / 2%（修正卷尺参考点后）

**历史上的"距离矛盾"真相**：卷尺 0 点顶在机身上量，而光学中心在前伸 8cm 的
镜头里——所有读数统一偏小 ~8cm，识别系统从来没有错。机器人上使用时，
该偏移由相机→本体的外参标定（TF 安装位姿）自然吸收。
