# QUTMS Driverless × EUFS — 话题、节点与 TF 清单（治理版）

> **生成方式**：话题/服务/TF 条目由 `scripts/topic_inventory.py` 对 `src/` 全仓
> `create_publisher / create_subscription / create_service / create_client`
> 调用点 grep 半自动生成，参数派生话题名按各节点 `declare_parameter` 默认值与
> `config/*.yaml`、`launch/*.py` 覆盖值人工解析；frame 约定与备注为人工编写。
> **基线**：commit `7ebc7dd`（= `074c38f` + 相机 mount 默认值 0.9/−0.21）。
> **生成日期**：2026-11-02。对账记录见文末 §7。
> 本文档取代初版（8c8488f，LiDAR 时代）；旧链条目以〔旧链〕标注，死包见各包
> README 的 UNUSED/LEGACY 标注（governance phase 4）。

---

## 0. 三条运行链

| 链 | 入口 launch | 感知 | 建图 | 规划 | 控制 |
|---|---|---|---|---|---|
| **sim** | `qutms_eufs_bridge/launch/yolo_autonomous_loop.launch.py` | EUFS Gazebo 相机 → yolo_detector_node | cone_mapper_node | urinay(+bridge)/ft/skidpad | path_tracker → sim 插件 |
| **bench/real** | `yolo_cone_detection/launch/zed_real_pipeline.launch.py` | ZED 2i → yolo_detector_node | cone_mapper_node | 同上（按 mission） | path_tracker → canbus |
| **skidpad mission** | `vehicle_bringup/launch/mission_launch/skidpad.launch.py`（内含 zed_real_pipeline） | 同 real | cone_mapper_node | skidpad_planner + warmup | path_tracker |

旧 nav 定位链（slam_toolbox + EKF + base_footprint）仅 trackdrive/ebs_test 两个
mission launch 仍引用，见 `nav_bringup` README 的 LEGACY 标注。

## 1. 主链话题（感知 → 建图 → 规划 → 控制）

QoS 缩写：`LATEST` = `driverless_common.common.QOS_LATEST`（BEST_EFFORT /
KEEP_LAST depth 1 / VOLATILE）；`ALL` = `QOS_ALL`（RELIABLE / depth 100）；
`d10` = 默认 QoS（RELIABLE / depth 10）；`d1` = 默认 depth 1。

| 话题 | 类型 | 生产者 | 消费者 | QoS | frame 约定 | 备注 |
|---|---|---|---|---|---|---|
| `/zed2i/zed_node/rgb/image_rect_color` | sensor_msgs/Image | EUFS sim ZED 插件 | yolo_detector_node | BEST_EFFORT d1 | `zed2i_left_camera_frame_optical` | **sim 链**；real 链经 launch 覆盖为 wrapper v5 名 ↓ |
| `/zed2i/zed_node/rgb/color/rect/image` | sensor_msgs/Image | ZED wrapper v5（实车） | yolo_detector_node | BEST_EFFORT d1 | 同上 | `zed_real_pipeline.launch.py` 默认值 |
| `/zed2i/zed_node/depth/depth_registered` | sensor_msgs/Image (32FC1) | sim 插件 / ZED wrapper | yolo_detector_node | BEST_EFFORT d1 | 光学帧 | 检测器缓存 8 帧按时间戳就近配对 |
| `/zed2i/zed_node/rgb/camera_info`（sim）<br>`/zed2i/zed_node/rgb/color/rect/image/camera_info`（real） | sensor_msgs/CameraInfo | 同上 | yolo_detector_node | LATEST | — | fy/内参来自此处，勿手填 |
| `/yolo/annotated_image` | sensor_msgs/Image | yolo_detector_node（`publish_annotated:=true`） | 调试/RViz | LATEST | 同图像 | 2D 框调试图 |
| `slam/cone_detection` | driverless_msgs/ConeDetectionStamped | yolo_detector_node | cone_mapper_node；node_topic_to_csv（可选） | LATEST 双侧 | header.frame_id=`track`；cone.location 在 track | 逐帧检测，**无时序记忆**；cones_with_cov 带对角协方差 |
| `/mapping/cone_map` | driverless_msgs/ConeDetectionStamped | cone_mapper_node | node_cone_id_mapper、node_ft_planner（cone_topic）、node_skidpad_planner、node_topic_to_csv（可选） | LATEST 双侧 | track | 静态世界门输出：仅 established 且 obs≥min_observations |
| `/mapping/cone_map_viz` | visualization_msgs/MarkerArray | cone_mapper_node | RViz | d1 | track | 含 tentative 灰球 + 坐标文本 |
| `cone_mapper/clear_map`（服务） | std_srvs/Trigger | —（bench 清图） | cone_mapper_node | — | — | 见 §4 |
| `/urinay/cone_map` | driverless_msgs/ConeWithIdArray | node_cone_id_mapper | node_urinay_path_bridge、node_planner_arbiter | LATEST 双侧 | track | 带全局 id 的锥桶地图 |
| `/AS/P/pathlimits/partial` | driverless_msgs/PathLimits | urinay_exec（C++，参数 `output_partial_topic`） | node_urinay_path_bridge | param QoS | track | urinay 部分限界 |
| `/AS/P/pathlimits/full` | driverless_msgs/PathLimits | 同上（`output_full_topic`） | node_urinay_path_bridge | param QoS | track | urinay 全限界 |
| `/planning/midline_path` | nav_msgs/Path | node_urinay_path_bridge | node_planner_arbiter（active_topic，影子记录用） | LATEST 双侧 | track | 中线（urinay） |
| `/shadow/ft/midline_path`、`/shadow/urinay/midline_path` | nav_msgs/Path | node_ft_planner（影子配置） | node_planner_arbiter（shadow_topic） | LATEST 双侧 | track | 影子规划器输出，仅供 arbiter 对比 |
| `/planning/path_with_speeds` | driverless_msgs/PathWithSpeeds | node_ft_planner:213（主）、node_urinay_path_bridge、node_skidpad_planner | node_path_tracker、node_safety_watchdog、node_topic_to_csv（可选） | LATEST 双侧 | track | **控制的唯一路径输入**；同一话题三个潜在生产者，mission 决定谁在跑 |
| `/planning/blue_bounds` / `/planning/yellow_bounds` | nav_msgs/Path | node_urinay_path_bridge、node_ft_planner | RViz/调试 | LATEST | track | 左右边界可视化 |
| `/planning/boundary_grid` | nav_msgs/OccupancyGrid | node_ft_planner | RViz/调试 | LATEST | track | 走廊栅格（ft） |
| `/planning/boundary_grid_metadata` | nav_msgs/MapMetaData | node_ft_planner | RViz/调试 | LATEST | — | 栅格元数据 |
| `planning/skidpad_full_path` | nav_msgs/Path | node_skidpad_planner | RViz/调试 | LATEST | track | skidpad 全局参考线 |
| `planning/skidpad_state` | std_msgs/String | node_skidpad_planner | node_skidpad_handler、skidpad_yolo_warmup（CLI 参数） | LATEST 双侧 | — | skidpad 状态机广播 |
| `planning/skidpad_markers` | visualization_msgs/MarkerArray | node_skidpad_planner | RViz | LATEST | track | skidpad 调试标记 |
| `/vehicle/velocity` | std_msgs/Float32 | EUFS sim 插件（sim）**未核** / 实车待接（canbus 组合速度为 `vehicle/combined_velocity`，见 §3） | node_ft_planner、node_urinay_path_bridge、node_path_tracker | LATEST 双侧 | — | m/s；**实车生产者未接通**（F12 定位缺口同级，仅登记） |
| `/vehicle/steering_angle` | std_msgs/Float32 | canbus 组件（C++，实车回读） | node_path_tracker | d10 | — | 前轮实际转角回读 |
| `/vehicle/wheel_speed1..3` | std_msgs/Float32 | canbus 组件（C++） | 调试 | d10 | — | 轮速 |
| `/control/driving_command` | driverless_msgs/AckermannDriveStamped | node_path_tracker、node_safety_watchdog、node_sine、pushcart_cmdvel_to_ackermann（相对名） | steering_actuator（C++，QOS_ALL）、velocity_controller（C++，QOS_ALL）、node_display | 见左 | — | **控制唯一出口**；C++ 侧 RELIABLE 订阅 |
| `/control/accel_command` | driverless_msgs/AckermannDriveStamped | node_sine | velocity_controller（C++） | d1 | — | 加速度测试激励 |
| `/cmd_vel` | geometry_msgs/Twist | nav 链 remap（旧链 `control/nav_cmd_vel`） | pushcart_cmdvel_to_ackermann | d1 | — | 推车调试桥 |

## 2. 系统管理 / 车辆状态

| 话题 | 类型 | 生产者 | 消费者 | QoS | 备注 |
|---|---|---|---|---|---|
| `system/av_state` | driverless_msgs/AVStateStamped | node_vehicle_supervisor、node_vehicle_supervisor_spoof | qutms_test_gui（vehicle_state）、node_display | d1 | AS 状态机 |
| `system/ros_state` | driverless_msgs/ROSStateStamped | 两个 supervisor 同上 | GUI | d1 | ROS 侧状态 |
| `system/as_status` | driverless_msgs/State | node_state_controller（C++？见注） | GUI | d1 | AS 状态（Inspection/EBS 等）；生产者为 terminal_control 状态控制器 |
| `/system/laps_completed` | std_msgs/UInt8 | node_vehicle_supervisor（等） | GUI | d1 | 圈数 |
| `/system/r2d` | std_msgs/Bool | node_controller（terminal_control） | supervisor 链 | d1 | R2D 信号 |
| `/system/mission_select` | std_msgs/UInt8 | GUI / node_display | node_state_controller | d1 | mission 选择 |
| `/system/reset` | driverless_msgs/Reset | GUI | node_skidpad_planner、node_path_tracker、node_safety_watchdog | LATEST | 复位（含计数器） |
| `/system/shutdown` | driverless_msgs/Shutdown | GUI | 同上三节点 | LATEST | 优雅停机 |
| `on_request/mission` `on_request/state` `on_request/lap` | std_msgs/UInt8 | GUI | node_state_controller | d1 | GUI→状态机请求 |

## 3. 底盘 / 执行器（C++ 组件，实车）

| 话题 | 类型 | 生产者 | 消费者 | QoS | 备注 |
|---|---|---|---|---|---|
| `/can/canbus_rosbound` | driverless_msgs/Can | node_socket_translator（canbus 组件） | component_canbus_translator | d10 | SocketCAN → ROS |
| `/can/canopen_rosbound` | driverless_msgs/Can | 同上 | canbus 组件 | d10 | CANopen 帧 |
| `can/canbus_carbound` | driverless_msgs/Can | component_canbus_translator | node_socket_translator | d10 | ROS → SocketCAN |
| `vehicle/combined_velocity` | std_msgs/Float32 | component_canbus_translator | velocity_controller | d10 | 轮速组合速度 |
| `vehicle/torque_request` | std_msgs/Float32 | velocity_controller | canbus 组件 | d10 | 扭矩请求 |

## 4. 服务

| 服务 | 类型 | 提供者 | 调用者 |
|---|---|---|---|
| `launch/system` | std_srvs/SetBool | node_system_watcher（system.launch） | GUI / node_vehicle_supervisor |
| `launch/skidpad` | std_srvs/SetBool | node_skidpad_watcher / handler | GUI |
| `launch/trackdrive` | std_srvs/SetBool | node_trackdrive_watcher | GUI / supervisor |
| `launch/ebs` | std_srvs/SetBool | node_ebs_watcher | GUI / supervisor |
| `launch/inspection` | std_srvs/SetBool | node_inspection_watcher | GUI / supervisor |
| `cone_mapper/clear_map` | std_srvs/Trigger | cone_mapper_node | bench 运维（清幽灵） |
| `bag/start` / `bag/stop` | TriggerBagRecord / Trigger | rosbag_creator | GUI |
| `/race_car_model/command_mode` | （EUFS 服务） | Gazebo sim 插件 | node_vehicle_supervisor 等 |
| `/system/reset_car_pos` | （EUFS 服务） | Gazebo sim 插件 | supervisor / GUI |

## 5. TF 树

### 5.1 sim 链（yolo_autonomous_loop）

| 边（父→子） | 发布者 | 静/动 | 数值来源 |
|---|---|---|---|
| `track → odom` | EUFS Gazebo race car 插件（`gazebo_ros_race_car.cpp:441`） | 动态 | 恒等（`pluginParams.yaml` map/odom 帧）+ 噪声状态置零于 odom |
| `odom → base_link` | 同上（`:431`） | 动态 | 带噪声的车辆位姿（noise_config） |
| `base_link → zed2i_camera_link → 左右相机帧 → optical 帧` | robot_state_publisher（eufs_launcher 装载 `eufs_racecar/urdf/robot.urdf.xacro`，根 `base_link`） | 静态 | URDF；**optical 关节为 identity（F10，见 §6）** |
| 轮/IMU/GPS 等 URDF 关节 | 同上 robot_state_publisher | 静态 | `eufs_racecar/urdf/*.xacro`（wheelbase 单源见 `wheels.urdf.xacro`） |

### 5.2 bench/real 链（zed_real_pipeline.launch.py）

| 边 | 发布者 | 静/动 | 数值来源 |
|---|---|---|---|
| `track → base_link` | static_transform_publisher `track_to_base_link`（launch 参数 `publish_track_tf`，默认 true） | 静态 | 恒等；**上车跑定位时必须置 false，让定位发布该边** |
| `base_link → zed2i_camera_link` | static_transform_publisher `base_link_to_camera_link`（`publish_camera_tf`） | 静态 | launch 参数 `cam_x/y/z/roll/pitch/yaw`（当前默认 = 2026-11-02 bench 实测 0.9 m / −0.21 rad）；实车装车值需按实测改 |
| `zed2i_camera_link → … → 左右相机帧 → optical 帧` | ZED wrapper 的 robot_state_publisher（`zed_camera.launch.py` 参数 `publish_urdf`，默认 true） | 静态 | wrapper 内置 URDF；**实车 optical 关节为真旋转**（见 §6 F3/F10） |

### 5.3 system/旧链（system.launch.py / nav_bringup）

| 边 | 发布者 | 静/动 | 数值来源 |
|---|---|---|---|
| `base_footprint → …车体关节…` | robot_state_publisher（`vehicle_urdf/launch/robot_description.launch.py`，`base_frame` 默认 **base_footprint**） | 静态 | `vehicle_urdf/urdf/vehicle.urdf.xacro`（`${base_frame}` 参数） |
| `odom → base_footprint` 等 | nav_bringup 定位链（EKF/slam_toolbox，**旧链，仅 trackdrive/ebs_test 引用**） | 动态 | `nav_bringup/config/localisation_params.yaml`（含布里斯班磁偏角等遗产，见该包 README） |

## 6. frame 约定（接缝登记）

### F1 — base_link / base_footprint 双名制

两套根帧并存，**场景划分**：

| 场景 | 根帧 | 谁发布 `track→根` | 谁查询 |
|---|---|---|---|
| 独立感知/规划链（yolo_autonomous、zed_real_pipeline、skidpad.launch） | **base_link** | zed_real_pipeline 静态 TF（bench）/ EUFS 插件（sim） | yolo_detector_node（TF 查 `track←body`）、ft_planner:156/351（默认 `base_frame=base_link`） |
| system 整车链（system.launch + 旧 nav 定位） | **base_footprint** | 旧 nav 链 | vehicle_bringup 的 system/push_log 启动与 mission handler（查 `track→base_footprint`） |

**风险**：同车上两链混跑时 mission handler 查 `track→base_footprint` 会失败或
错位。改名收敛方案见 `docs/frame_unification_plan.md`（草案，未执行）。

### F3 — ZED wrapper `publish_urdf` 是实车链命脉

`publish_tf:=false` 只关闭**位姿跟踪 TF**（wrapper 视觉里程计的
`odom→camera`），而相机内部帧（`…_left_camera_frame`、`…_optical`）依赖
wrapper 的 robot_state_publisher（`publish_urdf:=true`，默认 true）。
**任何启动配置都不得关 `publish_urdf`**，否则检测器查不到 `track←optical`
TF，逐帧跳过（"TF unavailable"）。`zed_real_pipeline.launch.py` 与
`mission_launch/skidpad.launch.py` 依赖此行为。

### F10 — 唯一坐标适配点：yolo_detector_node 内部

sim URDF 的 optical 关节是 identity，实车 wrapper 是真旋转（−90/0/−90），
两者同名不同值。检测器**不读 optical TF**，而是把反投影坐标在代码内从光学
轴序手工换轴到 body 轴序（X 前 Y 左 Z 上），再查 `track←body` TF（body 帧名
由 `camera_frame.replace("_optical","")` 推导）。**这是全仓唯一坐标适配点，
新代码不得另设适配或直接信任 optical TF**。

## 7. 对账记录（governance phase 1）

`python3 scripts/topic_inventory.py`（074c38f+7ebc7dd）：

- 字面量调用点 99 处 → 去重 58 个字符串；其中 6 个为解析噪声
  （`=`, `asynchronously`, `Default`, `SetBool`, `Trigger`, 空串——来自
  C++ 单行声明与 docstring 误配），**实际字面量话题 52 个**；
- 参数派生调用点约 30 处 → 按默认值解析后 20 个话题，其中 13 个与字面量
  重叠（`slam/cone_detection`、`/mapping/cone_map`、`/planning/*`、
  `/system/shutdown`、`/system/reset` 等），**净新增 7 个**：
  `/urinay/cone_map`、`/shadow/ft|urinay/midline_path`、
  `/AS/P/pathlimits/partial|full`、`planning/skidpad_full_path`、
  `planning/skidpad_state`、`planning/skidpad_markers`；
- 前导斜杠差异（`slam/cone_detection` vs `/slam/cone_detection` 等 5 对）
  为相对/绝对名等价写法，文档各合并为 1 行；
- **对账总数：52 + 7 = 59 个真实话题**；本文档 §1–§3 合计 62 行（含
  `/vehicle/wheel_speed1..3` 合并行展开为 4 个、camera_info 双名合并行），
  覆盖 ≥ 59，差额行均为合并行，无遗漏；
- GUI 的 `topic_monitor` / `node_topic_to_csv` 为任意话题订阅工具（话题名
  来自 CLI 参数），不计固定接口；
- skidpad_yolo_warmup 的 warmup/production 状态话题为 CLI 参数传入（缺省接
  `planning/skidpad_state`），并入该行备注。

**已验证**：以上条目均对应代码调用点；**未验证**：EUFS sim 插件
`/vehicle/velocity` 生产者（eufs_sim 不在本仓，未亲测）。
