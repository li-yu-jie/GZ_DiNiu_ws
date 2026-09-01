# 地牛自动物流调度系统 (FMS) 操作说明书

## 1. 架构简介
FMS (Fleet Management System) 模块整合了 **Nav2 全局导航** 和 **AprilTag 视觉对齐** 算法，通过全新的 `DiNiu_UI` 下发任务，实现了工业级的“取货 -> 视觉纠偏 -> 插取 -> 卸货”全自动化物流流程。

## 2. 启动流程（必须按顺序）
要让自动物流系统跑起来，您需要开启四个终端容器。

**终端 1：底层与导航**
```bash
ros2 launch diuniu_n10_nav n10_nav_all.launch.py
```

**终端 2：AprilTag 视觉对齐（关键！）**
> 注意：如果不开启此节点，小车到达取货点后将卡死在原地等待视觉服务。
```bash
ros2 launch diuniu_apriltag apriltag.launch.py align:=true
```

**终端 3：DiNiu_UI 网页后台**
```bash
ros2 launch diuniu_web_ui web_ui.launch.py
```

**终端 4：FMS 任务调度大脑**
```bash
ros2 run diuniu_task_manager fms_node
```

## 3. 网页端发车
1. 打开浏览器访问 `http://127.0.0.1:8000`
2. 在左侧面板找到 **【自动物流调度】**
3. 您可以通过下拉菜单快速选择已存的**导航点**（例如：1号、6号），也可以直接点击“📍地图选点”在地图上任意点选。
4. 选好取货点和卸货点后，点击 **🚀 立即发车**。

## 4. 部署依赖与安全模型

**Python 依赖**（新机/容器首次部署）：
```bash
pip3 install -r ~/GZ_DiNiu_ws/src/diuniu_web_ui/requirements.txt
# websockets 不装也能跑（自动回退包内 _vendor 副本），其余为必需
```

**账号**：首次启动自动生成 admin 初始密码并**只在日志里打印一次**，请立即登录修改。

**安全模型**：

- rosbridge 只监听 `127.0.0.1:9090`，浏览器经 FastAPI `/ws/rosbridge` **JWT 鉴权代理**接入，未登录无法建立任何 ROS 连接
- REST API 全部要登录，角色三级：viewer（看）/ operator（导航启停）/ admin（建图改图账号）
- 登录限流：同一 IP 连续失败 5 次锁 60 秒
- 跨网段/不可信网络务必开 HTTPS：`export DIUNIU_TLS_CERT=/path/cert.pem DIUNIU_TLS_KEY=/path/key.pem` 后启动
- 注意：同一 DDS 域内直接用 ROS2 CLI/节点仍可无鉴权访问（SROS2 才能根治），本系统只守 Web 入口，请保证车间网络可信

## 5. 常见问题排查 (Troubleshooting)

- **现象：车子到了取货点就不动了，终端也不报错。**
  **原因**：没有启动终端2的视觉对齐（`apriltag.launch.py align:=true`）。FMS大脑正在无限期等待唤醒视觉对齐服务。
  
- **现象：网页点击没有反应，或者地图显示错误。**
  **原因**：包名冲突或网页未重新编译。
  **解决**：只需进到源码目录更新：
  ```bash
  cd ~/GZ_DiNiu_ws/src/diuniu_web_ui && sh deploy.sh
  ```
