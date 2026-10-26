# QUTMS Driverless x EUFS Simulator Test & Control GUI 使用教程

## 1. 概述

本 GUI 是 QUTMS Driverless 与 EUFS Simulator 联调测试的统一管理界面。

**技术栈**：Python + PySide6 + rclpy

**功能**：
- 仿真启动/停止/重启
- ROS2 节点与 Topic 实时监视
- 车辆状态与控制指令实时显示
- 任务选择与启动
- 急停控制
- 统一日志（GUI + 文件）

**原则**：
- 不修改任何控制算法、感知算法、规划算法
- 不伪造 ROS2 Topic、消息或 Service
- 所有接口均从项目源码确认

---

## 2. 环境要求

| 依赖 | 说明 |
|------|------|
| ROS 2 | Humble / Jazzy（与你的工作区发行版一致） |
| Python 3 | >= 3.10 |
| PySide6 | `pip install PySide6` |
| colcon | ROS 2 标准构建工具 |
| Gazebo | eufs_sim 所需仿真环境（simulation/） |

### 2.1 安装 PySide6

```bash
pip install PySide6
```

### 2.2 确认工作区结构

```
QUTMS_Driverless-master/
├── simulation/        # eufs_sim
└── system/
    └── src/
        ├── common/
        ├── control/
        ├── hardware/
        ├── navigation/
        ├── operations/
        ├── perception/
        ├── qutms_eufs_bridge/
        └── tools/
            └── qutms_test_gui/    ← 本 GUI
```

---

## 3. 编译与启动

### 3.1 编译 GUI 包

```bash
cd system
colcon build --packages-select qutms_test_gui
```

### 3.2 Source 环境

```bash
source install/setup.bash
```

### 3.3 启动 GUI

**方式一**（推荐，直接运行）：

```bash
ros2 run qutms_test_gui qutms_test_gui
```

**方式二**（通过 launch 文件）：

```bash
ros2 launch qutms_test_gui qutms_test_gui.launch.py
```

### 3.4 验证启动成功

启动后应看到标题为 **"QUTMS Driverless x EUFS Simulator"** 的窗口，包含 6 个标签页。

GUI 启动后会自动：
1. 初始化 rclpy 并创建名为 `qutms_test_gui_ros_manager` 的隐藏节点
2. 开始轮询 ROS2 graph（每 2 秒）
3. 订阅车辆状态和控制指令 Topic
4. 监控关键 Topic 的发布频率

如果 ROS2 可用，Dashboard 右侧 **ROS2** 状态应显示为绿色 **CONNECTED**。

---

## 4. 页面功能详解

### 4.1 Dashboard（总览）

**布局**：左 / 中 / 右 三栏 + 底部状态栏。

#### 左栏 — 系统控制

| 按钮 | 功能 |
|------|------|
| Start Simulator | 启动仿真（执行 `ros2 launch qutms_eufs_bridge minimal_control_loop.launch.py`） |
| Stop Simulator | 停止仿真进程 |
| Restart Simulator | 重启仿真进程 |

启动后状态栏会显示 `Simulator: RUNNING`。

#### 左栏 — 任务控制

| 按钮 | 功能 |
|------|------|
| Manual | 选择 Manual 任务（mission_id=1） |
| Autonomous | 选择 Trackdrive 任务（mission_id=4） |
| Start Mission | 启动 Trackdrive 任务 |
| Pause Mission | **NOT CONFIGURED**（代码库无现成 pause 接口，仅记录日志） |
| Reset Mission | 重置任务（发布 `/system/reset` + 调用 `/system/reset_car_pos` 和 `/system/reset_cones` Service） |
| EMERGENCY STOP | 紧急停车（见 4.6） |

#### 中栏 — 车辆状态

| 字段 | 数据来源 | 单位 |
|------|---------|------|
| Speed | `/vehicle/velocity` | m/s |
| Steering | `/vehicle/steering_angle` | 度 |
| Throttle | `/control/driving_command.drive.speed` | m/s |
| Brake | `/control/driving_command.drive.acceleration` | m/s²（负值=制动） |
| X | `/odometry/sbg_ekf.pose.pose.position.x` | m |
| Y | `/odometry/sbg_ekf.pose.pose.position.y` | m |
| Yaw | `/odometry/sbg_ekf` 四元数转换 | rad |

未收到数据时显示 `NOT AVAILABLE`。

#### 右栏 — ROS2 与子系统状态

| 字段 | 说明 |
|------|------|
| Nodes | 当前 ROS2 graph 中的节点数 |
| Topics | 当前 Topic 数 |
| Services | 当前 Service 数 |
| Actions | 当前 Action 数 |
| TF | NOT TESTED（需运行验证） |
| Controller | 基于 `/system/ros_state.good_to_go` |
| Perception | 基于 `/system/ros_state.lidar_operational` |
| Navigation | 基于 `/system/ros_state.planning` |

---

### 4.2 Nodes（节点监视）

显示当前 ROS2 graph 中所有节点的实时列表。

| 列 | 说明 |
|----|------|
| Node Name | 节点全名（含命名空间） |
| Namespace | 命名空间 |
| Status | RUNNING（存在即显示） |

**使用场景**：
- 确认 eufs 节点是否已启动
- 确认 QUTMS 控制器节点是否在运行
- 排查节点崩溃或未启动问题

---

### 4.3 Topics（Topic 监视）

显示当前 ROS2 graph 中所有 Topic 的实时列表。

| 列 | 说明 |
|----|------|
| Topic | Topic 名称 |
| Message Type | 消息类型 |
| Publishers | 发布者数量 |
| Subscribers | 订阅者数量 |
| Freq (Hz) | 发布频率（1 秒窗口统计） |

GUI 会自动监控以下 6 个关键 Topic 的频率和新鲜度：

| Topic | 消息类型 |
|-------|---------|
| `/vehicle/velocity` | std_msgs/Float32 |
| `/vehicle/steering_angle` | std_msgs/Float32 |
| `/odometry/sbg_ekf` | nav_msgs/Odometry |
| `/control/driving_command` | ackermann_msgs/AckermannDriveStamped |
| `/system/av_state` | driverless_msgs/AVStateStamped |
| `/system/ros_state` | driverless_msgs/ROSStateStamped |

超过 2 秒未收到消息的 Topic 标记为 **STALE**。

**使用场景**：
- 确认控制指令是否在发布（`/control/driving_command` 应有 1 publisher + 1 subscriber）
- 检查车辆状态 Topic 是否正常更新
- 排查消息类型不匹配问题

---

### 4.4 Vehicle（车辆状态）

详细显示车辆状态和控制指令，包含最后更新时间和 STALE 检测。

**Vehicle State 区域**：

| 字段 | 说明 |
|------|------|
| Speed | 车辆速度（m/s） |
| Steering | 转向角（度） |
| X / Y | 车辆位置（m） |
| Yaw | 航向角（rad） |
| Last update | 距上次收到数据的时间（秒），超过 1 秒变红 |

**Control Command 区域**：

| 字段 | 说明 |
|------|------|
| Steering | `/control/driving_command.drive.steering_angle`（度） |
| Speed | `/control/driving_command.drive.speed`（m/s） |
| Acceleration | `/control/driving_command.drive.acceleration` |
| Last update | 距上次收到指令的时间（秒），超过 1 秒变红 |

---

### 4.5 Mission（任务控制）

**任务选择**：

| 按钮 | Mission ID | 说明 |
|------|-----------|------|
| None | 0 | 无任务 |
| Manual Driving | 1 | 手动驾驶 |
| Inspection | 2 | 巡检 |
| EBS Test | 3 | 紧急制动测试 |
| Trackdrive | 4 | 圈速赛 |

点击任务按钮会发布 `/system/mission_select`（std_msgs/UInt8）。

**System State 区域**：

| 字段 | 来源 |
|------|------|
| AV Mode | `/system/av_state.mode`（0=MANUAL, 1=AUTONOMOUS） |
| AV State | `/system/av_state.state`（0=NOT_READY, 1=START_MISSION, 2=DRIVING, 3=END） |
| Mission | `/system/av_state.mission` |
| Good to go | `/system/ros_state.good_to_go` |
| Laps | `/system/ros_state.lap_count` |

---

### 4.6 Emergency Stop（紧急停车）

**红色 EMERGENCY STOP 按钮位于 Dashboard 左栏和 Mission 页面。**

点击后 GUI 会执行两步操作：

1. 发布 `Shutdown{emergency_shutdown: true}` 到 `/system/shutdown`
2. 发布零速 `AckermannDriveStamped{speed: 0, steering_angle: 0, acceleration: 0}` 到 `/control/driving_command`

> **注意**：eufs RaceCarPlugin 的 `/system/as_status` 订阅已被注释（源码第 86-87 行），`State::EMERGENCY` 不会直接让 eufs 车辆停车。真正让车辆停车的是第 2 步的零速指令。

---

### 4.7 Logs（日志）

显示统一格式的日志：

```
[时间] 级别  模块       消息
```

| 颜色 | 级别 |
|------|------|
| 灰色 | DEBUG |
| 绿色 | INFO |
| 橙色 | WARN |
| 红色 | ERROR |

日志同时写入文件 `logs/qutms_gui.log`。

**使用场景**：
- ROS2 连接问题
- Topic 不存在
- Node 不存在
- Launch 失败
- Process 崩溃
- 消息类型不匹配
- TF 缺失

---

## 5. 典型使用流程

### 5.1 首次联调测试

```
1. 编译整个工作区
   colcon build

2. Source 环境
   source install/setup.bash

3. 启动 GUI
   ros2 run qutms_test_gui qutms_test_gui

4. 确认 ROS2 状态为 CONNECTED

5. 点击 Start Simulator 启动仿真
   → 状态栏应显示 Simulator: RUNNING
   → Nodes 页应出现 eufs 相关节点
   → Topics 页应出现 /control/driving_command 等

6. 发布直线测试指令
   ros2 topic pub /control/nav_cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10

7. 在 Vehicle 页确认：
   → Speed 显示 ~0.5 m/s
   → X 坐标在变化
   → /control/driving_command 有数据更新

8. 测试急停
   → 点击 EMERGENCY STOP
   → 车辆应停止
   → Logs 页应显示急停操作记录

9. 点击 Stop Simulator 停止仿真
```

### 5.2 日常调试

```
1. 启动 GUI
2. 通过 Start Simulator 启动仿真
3. 切换到 Nodes 页确认所有预期节点在运行
4. 切换到 Topics 页确认关键 Topic 有正确的 publisher/subscriber
5. 切换到 Vehicle 页监控车辆实时状态
6. 如遇异常，查看 Logs 页定位问题
7. 测试结束后点击 Stop Simulator
```

### 5.3 故障排查

| 现象 | 排查步骤 |
|------|---------|
| ROS2: DISCONNECTED | 确认 `ros2 daemon` 正在运行；检查 ROS_DOMAIN_ID |
| Simulator: ERROR | 查看 Logs 页的 stderr 输出；确认 eufs_sim 包已编译 |
| Nodes 列表为空 | 确认仿真已启动；检查 `ros2 node list` |
| Topic Freq 显示 STALE | 确认发布者节点在运行；检查消息类型是否匹配 |
| Vehicle 全部 NOT AVAILABLE | 确认 euds 仿真已启动且 RaceCarPlugin 已加载 |
| EMERGENCY STOP 无效 | 检查 `/control/driving_command` 是否有 subscriber（euls RaceCarPlugin） |

---

## 6. 架构说明

### 6.1 线程模型

```
┌─────────────────────────────────────┐
│         Qt 主线程（GUI）            │
│  Dashboard / Nodes / Topics / ...   │
│  通过 Signal/Slot 更新 UI           │
└──────────┬──────────────────────────┘
           │
     ┌─────┴──────┐  ┌──────────────┐  ┌──────────────┐
     │ rclpy spin │  │ Graph 轮询   │  │ ProcessMgr  │
     │  线程       │  │ 线程(2s)    │  │ (QProcess)  │
     └────────────┘  └──────────────┘  └──────────────┘
           │
     ┌─────┴──────────────────────┐
     │  Topic 回调（rclpy 线程内） │
     │  vehicle_state / topic_monitor │
     └────────────────────────────┘
```

- **Qt 主线程**：只负责 UI 渲染和 Signal/Slot 派发，不执行任何阻塞操作
- **rclpy spin 线程**：在后台循环调用 `rclpy.spin_once()`，处理 Topic 回调
- **Graph 轮询线程**：每 2 秒调用 `get_node_names_and_namespaces()` 等获取 ROS2 graph 信息
- **ProcessManager**：使用 `QProcess`（非阻塞），捕获 stdout/stderr

### 6.2 包结构

```
qutms_test_gui/
├── package.xml           # ament_python 包声明
├── setup.py              # 入口点: qutms_test_gui = qutms_test_gui.main:main
├── setup.cfg
├── resource/qutms_test_gui
├── launch/
│   └── qutms_test_gui.launch.py
└── qutms_test_gui/
    ├── __init__.py
    ├── main.py           # 入口
    ├── logger.py         # 统一日志（文件 + GUI 信号）
    ├── gui/
    │   ├── main_window.py  # 主窗口，连接所有信号
    │   ├── dashboard.py    # Dashboard 页
    │   ├── nodes_page.py   # 节点监视页
    │   ├── topics_page.py  # Topic 监视页
    │   ├── vehicle_page.py # 车辆状态页
    │   ├── mission_page.py # 任务控制页
    │   └── logs_page.py    # 日志页
    ├── ros/
    │   ├── ros_manager.py    # rclpy 生命周期 + graph 轮询
    │   ├── topic_monitor.py  # Topic 频率监控
    │   └── vehicle_state.py  # 车辆状态订阅
    └── process/
        └── process_manager.py # 仿真进程管理（QProcess）
```

---

## 7. ROS2 接口清单

### 7.1 GUI 订阅的 Topic

| Topic | 消息类型 | 用途 |
|-------|---------|------|
| `/vehicle/velocity` | std_msgs/Float32 | 车辆速度 |
| `/vehicle/steering_angle` | std_msgs/Float32 | 转向角（度） |
| `/odometry/sbg_ekf` | nav_msgs/Odometry | 车辆位姿 |
| `/control/driving_command` | ackermann_msgs/AckermannDriveStamped | 控制指令 |
| `/system/av_state` | driverless_msgs/AVStateStamped | 自主状态 |
| `/system/ros_state` | driverless_msgs/ROSStateStamped | ROS 系统状态 |

### 7.2 GUI 发布的 Topic

| Topic | 消息类型 | 触发 |
|-------|---------|------|
| `/system/mission_select` | std_msgs/UInt8 | 任务选择按钮 |
| `/system/shutdown` | driverless_msgs/Shutdown | Emergency Stop |
| `/control/driving_command` | ackermann_msgs/AckermannDriveStamped | Emergency Stop（零速） |
| `/system/reset` | driverless_msgs/Reset | Reset Mission |

### 7.3 GUI 调用的 Service

| Service | 类型 | 触发 |
|---------|------|------|
| `/system/reset_car_pos` | std_srvs/Trigger | Reset Mission |
| `/system/reset_cones` | std_srvs/Trigger | Reset Mission |

### 7.4 未配置接口

| 接口 | 状态 | 说明 |
|------|------|------|
| Mission Pause | NOT CONFIGURED | 代码库无现成 pause 接口 |
| 独立 Brake Topic | NOT AVAILABLE | 通过 acceleration 负值表达 |
| `follow_path` Action | UNKNOWN | 需运行确认类型 |

---

## 8. 日志文件

日志文件位于 `logs/qutms_gui.log`，包含所有 DEBUG 及以上级别的日志。

格式：
```
2026-09-19 21:40:05,123 [INFO] qutms_test_gui.ros_manager: ROS2 connected
2026-09-19 21:40:06,456 [INFO] qutms_test_gui.main_window: Starting eufs simulator
2026-09-19 21:40:07,789 [WARN] qutms_test_gui.topic_monitor: Topic /vehicle/velocity: NO DATA
```

---

## 9. 常见问题

### Q: GUI 启动后 ROS2 显示 DISCONNECTED？

```bash
# 确认 ROS2 可用
ros2 node list

# 如果没有任何输出，可能 daemon 未运行
ros2 daemon start
```

### Q: 点击 Start Simulator 后立即显示 ERROR？

```bash
# 确认 qutms_eufs_bridge 包已编译
colcon list | grep qutms_eufs_bridge

# 手动测试 launch 命令
ros2 launch qutms_eufs_bridge minimal_control_loop.launch.py
```

### Q: Vehicle 页全部显示 NOT AVAILABLE？

- 确认 eufs 仿真已启动（状态栏显示 RUNNING）
- 确认 RaceCarPlugin 已加载（检查 Gazebo 控制台有无错误）
- 在 Topics 页检查 `/vehicle/velocity` 是否有 publisher

### Q: Emergency Stop 按钮没有效果？

- 在 Topics 页检查 `/control/driving_command` 是否有 subscriber（RaceCarPlugin）
- 确认 eufs 仿真正在运行
- 查看 Logs 页是否有发布失败的错误

### Q: 关闭 GUI 后仿真进程还在运行？

GUI 会在关闭时自动调用 `ProcessManager.stop()`。如果进程未正常退出：
```bash
# 查找残留的 ros2 launch 进程
ps aux | grep "ros2 launch"

# 手动终止
kill <PID>
```

---

## 10. 注意事项

1. **不要在 GUI 运行时手动执行 `ros2 launch`**：通过 GUI 的 Start/Stop 按钮管理仿真进程，避免进程冲突。
2. **首次使用前先单独测试 launch 命令**：确认 `ros2 launch qutms_eufs_bridge minimal_control_loop.launch.py` 能正常运行后再通过 GUI 启动。
3. **steering_angle 单位是度**：QUTMS 和 eufs 两端一致使用度，非弧度。
4. **急停是双通道**：`Shutdown` 通知系统 + 零速指令让车辆停车。
5. **Mission Pause 不可用**：当前代码库无 pause 接口，点击仅记录日志。
6. **日志文件位置**：`logs/qutms_gui.log`（相对于启动 GUI 时的工作目录）。
