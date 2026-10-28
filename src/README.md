# QUTMS Driverless × EUFS Simulator — ROS2 工作区使用文档

## 1. 这是什么

本 `src/` 目录包含 QUTMS Driverless 自动驾驶系统与 EUFS Simulator 仿真环境的全部 ROS2 包。
将此目录复制到 ROS2 工作区后，编译即可使用。

## 2. 包清单

### 仿真环境（7 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| eufs_config | ament_cmake | eufs 仿真配置（车辆参数、赛道等） |
| eufs_launcher | ament_python | eufs 仿真启动入口 |
| eufs_models | ament_cmake | Gazebo 车辆模型 |
| eufs_plugins | ament_cmake | Gazebo 插件（RaceCarPlugin、LiDAR、锥桶检测等） |
| eufs_racecar | ament_python | 赛道定义 |
| eufs_rqt | ament_python | eufs RQT 可视化工具 |
| eufs_tracks | ament_python | 赛道布局文件 |

### 消息与公共（4 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| driverless_msgs | ament_cmake | 自定义消息/服务定义（Cone、State、AVState 等） |
| driverless_common | ament_python | 公共工具库 |
| QUTMS_Embedded_Common | ament_cmake | 嵌入式公共代码（CAN 通信等） |
| rviz_marker_plugins | ament_cmake | RViz 自定义可视化插件 |

### 控制（3 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| controllers | ament_python | vel_to_ackermann 节点（Twist→Ackermann 转换） |
| velocity_controller | ament_cmake | 速度控制器（C++） |
| pushcart_cmdvel_to_ackermann | ament_cmake | 推车模式 cmd_vel 转换 |

### 感知（3 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| ground_plane_segmenter | ament_cmake | 地面分割（C++） |
| lidar_pipeline | ament_python | 锥桶检测管线 |
| perception_bringup | ament_cmake | 感知启动配置 |

### 导航（3 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| planners | ament_python | 路径规划器 |
| slam_gridmap | ament_python | SLAM 栅格地图转换 |
| nav_bringup | ament_cmake | Nav2 启动配置 |

### 运营与硬件（7 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| vehicle_bringup | ament_python | 车辆系统启动 + 状态机 |
| terminal_control | ament_python | 终端控制（任务选择、R2D 等） |
| vehicle_urdf | ament_cmake | 车辆 URDF 模型 |
| telem_bringup | ament_cmake | 遥测启动 |
| rosbag_creator | ament_python | 录包工具 |
| canbus | ament_cmake | CAN 总线翻译节点 |
| steering_actuator | ament_cmake | 转向执行器节点 |

### 桥接与 GUI（2 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| qutms_eufs_bridge | ament_python | QUTMS↔eufs 桥接 launch 包 |
| qutms_test_gui | ament_python | 测试管理 GUI（PySide6 + rclpy） |

### 视觉感知与规划（3 个）

| 包名 | 构建类型 | 说明 |
|------|---------|------|
| yolo_cone_detection | ament_python | ZED RGB/深度 + TCone-YOLO → 锥桶检测（含 TF 投影） |
| cone_mapping | ament_python | 多帧锥桶融合、颜色投票、过期淘汰 |
| urinay | ament_cmake (C++) | Delaunay 三角化 + 树搜索规划器，输出 PathLimits |

> ZED 相机相关的 `zed-ros2-*` 为第三方子模块，不计入上表。

## 3. 环境要求

| 依赖 | 版本 |
|------|------|
| ROS 2 | Humble（推荐）或 Jazzy |
| Gazebo | 与 ROS2 发行版对应 |
| Python | >= 3.10 |
| PySide6 | `pip install PySide6`（仅 GUI 需要） |
| colcon | ROS2 标准构建工具 |

## 4. 首次使用：最小控制闭环测试

最小控制闭环 = EUFS 仿真 + QUTMS vel_to_ackermann 控制器，不需要感知/规划/导航。

### 4.1 复制到 ROS2 环境

```bash
# 在 Ubuntu ROS2 机器上
mkdir -p ~/qutms_ws/src
# 将本 src/ 目录的全部内容复制到 ~/qutms_ws/src/
cp -r /path/to/src/* ~/qutms_ws/src/
```

### 4.2 安装 GUI 依赖

```bash
pip install PySide6
```

### 4.3 编译

```bash
cd ~/qutms_ws

# 第一步：先编译消息包（其他包依赖它）
colcon build --packages-select driverless_msgs
source install/setup.bash

# 第二步：编译最小闭环所需包
colcon build --packages-select \
  driverless_msgs \
  controllers \
  qutms_eufs_bridge \
  qutms_test_gui \
  eufs_launcher eufs_config eufs_models eufs_plugins eufs_racecar eufs_rqt eufs_tracks

# 第三步：source 环境
source install/setup.bash
```

### 4.4 启动方式

#### 方式 A：通过 GUI 启动（推荐）

```bash
ros2 run qutms_test_gui qutms_test_gui
```

GUI 打开后：
1. 点击 **Start Simulator** — 自动启动仿真 + 控制器
2. 确认 ROS2 状态变绿（CONNECTED）
3. 查看 Nodes 页确认节点出现
4. 查看 Topics 页确认 `/control/driving_command` 有 publisher + subscriber

#### 方式 B：命令行启动

```bash
# 终端 1：最小控制闭环
ros2 launch qutms_eufs_bridge minimal_control_loop.launch.py

# 或：YOLO 视觉自动驾驶闭环（planner: urinay | ft）
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py \
  track:=small_oval planner:=urinay
```

`tcone_yolo_path` 默认为空，YOLO 节点会自动查找：`QUTMS_TCONE_YOLO_PATH` 环境变量
→ 当前工作目录/包所在工作区的 `src/TCone-YOLO-main` → `~/qutms_ws/src/TCone-YOLO-main`。
也可显式传入 `tcone_yolo_path:=/abs/path/to/TCone-YOLO-main`。

### 4.5 测试车辆运动

```bash
# 终端 2：发送直线指令
ros2 topic pub /control/nav_cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10

# 终端 3：查看车辆速度反馈
ros2 topic echo /vehicle/velocity

# 终端 4：查看控制指令
ros2 topic echo /control/driving_command
```

Gazebo 中车辆应开始直线移动，`/vehicle/velocity` 应显示 ~0.5 m/s。

### 4.6 停止

- 命令行：`Ctrl+C`
- GUI：点击 **Stop Simulator**

## 5. 完整系统编译

如果需要完整的感知→规划→控制闭环：

```bash
cd ~/qutms_ws

# 先编译消息和公共包
colcon build --packages-select driverless_msgs driverless_common
source install/setup.bash

# 编译全部
colcon build

source install/setup.bash
```

> 注意：`QUTMS_Embedded_Common` 可能需要 git submodule 初始化。
> 如果编译报错缺头文件，先执行：
> ```bash
> cd src/QUTMS_Embedded_Common
> git submodule update --init
> ```
> 如果不需要 CAN 总线功能，可以跳过：
> ```bash
> colcon build --packages-skip QUTMS_Embedded_Common canbus steering_actuator state_control
> ```

## 6. 控制闭环数据流

```
/control/nav_cmd_vel (geometry_msgs/Twist)
      │
  vel_to_ackermann_node       ← controllers 包
      │
/control/driving_command (ackermann_msgs/AckermannDriveStamped)
      │
  eufs RaceCarPlugin          ← eufs_plugins 包
      │
  Gazebo 车辆运动
      │
  /vehicle/velocity (Float32)          → 车辆速度
  /vehicle/steering_angle (Float32)    → 转向角（度）
  /odometry/sbg_ekf (Odometry)         → 车辆位姿
```

## 6.1 YOLO 视觉闭环数据流

```
ZED2i RGB + depth
      │
yolo_cone_detection/yolo_detector_node      → slam/cone_detection
      │  （RGB/深度按时间戳配对，TF 用图像时间戳）
cone_mapping/cone_mapper_node               → /mapping/cone_map
      │  （多帧融合 + 颜色投票 + 输入静默超时）
planners/cone_id_mapper_node                → /urinay/cone_map
      │  （去重 + 全局最近邻 ID 关联）
urinay_exec (C++)                           → /AS/P/pathlimits/{partial,full}
      │
planners/urinay_path_bridge_node            → /planning/path_with_speeds
      │  （周期样条 + 三段速度规划）
controllers/path_tracker_node               → /control/driving_command
      │
controllers/safety_watchdog_node            → 路径/TF 超时急停
```

规划器可切换：`planner:=urinay`（默认，颜色无关）或 `planner:=ft`（FaSTTUBe，
基于蓝/黄锥桶边界）。两者都经过 [planners/config](planners/config) 中的参数配置。

## 7. GUI 功能说明

GUI 包含 6 个页面：

| 页面 | 功能 |
|------|------|
| Dashboard | 仿真控制（启动/停止/重启，YOLO 模式可选规划器与 YOLO 路径）、任务控制、车辆状态、ROS2 状态 |
| Nodes | 实时 ROS2 节点列表 |
| Topics | 实时 Topic 列表（名称、类型、发布/订阅数、频率） |
| Vehicle | 车辆状态 + 控制指令（含 STALE 检测） |
| Mission | 任务选择/启动/重置 + Emergency Stop |
| Logs | 统一日志（时间/级别/模块/消息，颜色区分） |

## 8. GUI 使用的 ROS2 接口

### 订阅（只读）

| Topic | 消息类型 | 用途 |
|-------|---------|------|
| /vehicle/velocity | std_msgs/Float32 | 车辆速度 |
| /vehicle/steering_angle | std_msgs/Float32 | 转向角（度） |
| /odometry/sbg_ekf | nav_msgs/Odometry | 车辆位姿 |
| /control/driving_command | ackermann_msgs/AckermannDriveStamped | 控制指令 |
| /system/av_state | driverless_msgs/AVStateStamped | 自主状态 |
| /system/ros_state | driverless_msgs/ROSStateStamped | 系统状态 |

### 发布（控制）

| Topic | 消息类型 | 触发 |
|-------|---------|------|
| /system/mission_select | std_msgs/UInt8 | 任务选择按钮 |
| /system/shutdown | driverless_msgs/Shutdown | Emergency Stop |
| /control/driving_command | ackermann_msgs/AckermannDriveStamped | Emergency Stop（零速） |
| /system/reset | driverless_msgs/Reset | Reset Mission |

### Service 调用

| Service | 类型 | 触发 |
|---------|------|------|
| /system/reset_car_pos | std_srvs/Trigger | Reset Mission |
| /system/reset_cones | std_srvs/Trigger | Reset Mission |

## 9. Emergency Stop 说明

点击红色 **EMERGENCY STOP** 按钮后，GUI 执行两步操作：

1. 发布 `Shutdown{emergency_shutdown: true}` 到 `/system/shutdown`
2. 发布零速 `AckermannDriveStamped{speed: 0, steering_angle: 0, acceleration: 0}` 到 `/control/driving_command`

> 注意：eufs RaceCarPlugin 的 `/system/as_status` 订阅已被注释（源码第 86-87 行），
> `State::EMERGENCY` 不会直接让车辆停车。真正让车辆停车的是第 2 步的零速指令。

## 10. 常见问题

### 编译找不到 driverless_msgs？

```bash
# 必须先单独编译消息包并 source
colcon build --packages-select driverless_msgs
source install/setup.bash
# 然后再编译其他包
```

### QUTMS_Embedded_Common 编译报错？

```bash
# 跳过不需要的硬件包
colcon build --packages-skip QUTMS_Embedded_Common canbus steering_actuator
```

### GUI 启动报 No module named PySide6？

```bash
pip install PySide6
```

### 车辆不动？

```bash
# 检查控制指令 Topic 是否有订阅者（RaceCarPlugin）
ros2 topic info /control/driving_command
# 应显示至少 1 个 publisher + 1 个 subscriber

# 检查 vel_to_ackermann_node 是否在运行
ros2 node list | grep vel_to_ackermann
```

### eufs 编译需要什么 ROS2 发行版？

eufs_sim 声明支持 ROS2 Galactic。在 Humble 上可能需要少量修改。
如果遇到 Gazebo 插件 API 不兼容，请检查 Gazebo 版本。

### 日志文件在哪？

```
~/qutms_ws/logs/qutms_gui.log
```

（相对于启动 GUI 时的工作目录）

## 11. 三种 Launch 入口

| Launch 文件 | 作用 |
|------------|------|
| `qutms_eufs_bridge/minimal_control_loop.launch.py` | eufs + QUTMS 控制器（最小闭环） |
| `qutms_eufs_bridge/eufs_control_sim.launch.py` | 仅 eufs 仿真（无控制器） |
| `qutms_eufs_bridge/qutms_controller.launch.py` | 仅 QUTMS vel_to_ackermann 控制器 |

## 12. 快速验证清单

编译成功后，依次验证：

- [ ] `ros2 run qutms_test_gui qutms_test_gui` GUI 能启动
- [ ] GUI 中 ROS2 状态显示 CONNECTED
- [ ] 点击 Start Simulator 后 Nodes 页出现节点
- [ ] Topics 页出现 `/control/driving_command`
- [ ] 发送 Twist 指令后 Gazebo 车辆移动
- [ ] `/vehicle/velocity` 显示速度值
- [ ] GUI Vehicle 页 Speed 有实时数值
- [ ] 点击 EMERGENCY STOP 后车辆停止
- [ ] Logs 页有正常日志输出
