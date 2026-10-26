# QUTMS Driverless × EUFS Simulator — 话题与节点清单

> 本文档记录工作区内所有 ROS2 节点的名称、订阅/发布话题、消息类型与数据流方向，
> 供后续接入 YOLO 视觉处理板块时参考。

---

## 1. 系统总览

本项目是基于 ROS2 的 Formula Student Driverless（FSAC）无人驾驶赛车系统，
由 **QUTMS 自动驾驶栈** 与 **EUFS Gazebo 仿真环境** 组合而成。

### 核心数据流（感知 → 规划 → 控制）

```
┌──────────────┐   /velodyne_points   ┌─────────────────────┐   lidar/objects    ┌──────────────────┐
│  LiDAR/Camera │ ──────────────────▶ │ GroundPlaneSegmenter │ ─────────────────▶ │ LiDARDetector    │
│  (仿真/实车)  │                      │  (地面分割)          │                    │  (锥桶检测)       │
└──────────────┘                      └─────────────────────┘                    └────────┬─────────┘
                                                                                          │
                                                                                   lidar/cone_detection
                                                                                          │
                                                                                          ▼
┌──────────────┐   slam/occupancy_grid  ┌──────────────────────┐   slam/cone_detection  ┌──────────┐
│ SLAM Toolbox │ ─────────────────────▶ │ GridmapToConeDetection│ ────────────────────▶ │  Planner │
│  (栅格建图)   │                        │  (栅格→锥桶)          │                       │ (路径规划)│
└──────────────┘                        └──────────────────────┘                       └────┬─────┘
                                                                                            │
                                                                                   planning/midline_path
                                                                                            │
                                                                                            ▼
                                                                                  ┌──────────────────┐
                                                                                  │ vel_to_ackermann │
                                                                                  │  (Twist→Ackermann)│
                                                                                  └────────┬─────────┘
                                                                                           │
                                                                                  control/driving_command
                                                                                           │
                                                                                           ▼
                                                                                  ┌──────────────────┐
                                                                                  │  RaceCarPlugin   │
                                                                                  │  (Gazebo 车辆)    │
                                                                                  └──────────────────┘
```

### 仿真与实车话题命名差异

| 话题 | 仿真来源 | 实车来源 |
|------|---------|---------|
| `velodyne_points` | Gazebo LiDAR 插件 | Velodyne 驱动节点 |
| `lidar/cone_detection` | ConeDetectionPlugin（仿真感知） | LiDARDetector 节点 |
| `vision/cone_detection` | ConeDetectionPlugin（仿真相机） | **YOLO 视觉节点（待接入）** |
| `slam/occupancy_grid` | SLAM Toolbox | SLAM Toolbox |
| `imu/odometry` | SBG 驱动插件 | SBG 驱动 |
| `odometry/sbg_ekf` | RaceCarPlugin 直接发布 | SBG EKF |

---

## 2. 节点清单（按功能分组）

### 2.1 仿真环境（eufs_plugins）

#### 2.1.1 RaceCarPlugin（`gazebo_ros_race_car.cpp`）
Gazebo 车辆动力学插件，处理 Ackermann 控制指令并发布车辆状态。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/vehicle/wheel_twist` | `geometry_msgs/TwistWithCovarianceStamped` | 轮速合成 twist |
| Pub | `/vehicle/steering_angle` | `std_msgs/Float32` | 转向角（度） |
| Pub | `/vehicle/velocity` | `std_msgs/Float32` | 车辆速度（m/s） |
| Pub | `/zed2i/zed_node/odom` | `nav_msgs/Odometry` | 视觉里程计（30Hz） |
| Pub | `/odometry/sbg_ekf` | `nav_msgs/Odometry` | SBG EKF 里程计（50Hz） |
| Pub | `slam/car_pose` | `geometry_msgs/PoseWithCovarianceStamped` | SLAM 位姿（simulate_slam=true） |
| Pub | `/ground_truth/wheel_twist` | `geometry_msgs/TwistWithCovarianceStamped` | 真值轮速 twist |
| Pub | `/ground_truth/odom` | `nav_msgs/Odometry` | 真值里程计 |
| Pub | `/ground_truth/steering_angle` | `std_msgs/Float32` | 真值转向角 |
| Pub | `/ground_truth/velocity` | `std_msgs/Float32` | 真值速度 |
| Pub | `/joint_states/steering` | `sensor_msgs/JointState` | 转向关节状态 |
| Sub | `/control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 控制指令 |
| Srv | `/system/reset_car_pos` | `std_srvs/Trigger` | 重置车辆位置 |
| Srv | `/race_car_model/command_mode` | `std_srvs/Trigger` | 查询控制模式 |

#### 2.1.2 ConeDetectionPlugin（`gazebo_ros_cone_detection.cpp`）
仿真锥桶检测插件，可模拟 LiDAR、相机、SLAM 三种感知输出。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `ground_truth/global_map` | `driverless_msgs/ConeDetectionStamped` | 全局真值锥桶图（pub_gt=true） |
| Pub | `lidar/cone_detection` | `driverless_msgs/ConeDetectionStamped` | 仿真 LiDAR 锥桶检测 |
| Pub | `vision/cone_detection` | `driverless_msgs/ConeDetectionStamped` | **仿真相机锥桶检测（YOLO 接入目标话题）** |
| Pub | `slam/global_map` | `driverless_msgs/ConeDetectionStamped` | 仿真 SLAM 全局图 |
| Pub | `slam/local_map` | `driverless_msgs/ConeDetectionStamped` | 仿真 SLAM 局部图 |
| Srv | `/system/reset_cones` | `std_srvs/Trigger` | 重置锥桶位置 |

> 仿真相机检测配置（`pluginParams.yaml`）：
> - `camera_frame_id`: `zed2i_left_camera_optical_frame`
> - `camera_min_view_distance`: 0.5m，`camera_max_view_distance`: 10.0m
> - `camera_fov`: 1.91986 rad（110°）
> - `camera_detects_colour`: true

#### 2.1.3 GazeboRosLidar（`gazebo_ros_lidar.cpp`）
Gazebo 激光雷达传感器插件。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/velodyne_points` | `sensor_msgs/PointCloud2` | 原始点云 |
| Pub | `/scan` | `sensor_msgs/LaserScan` | 2D 激光扫描（enable_laserscan=true） |

#### 2.1.4 ZED2i 相机（URDF 中 `libgazebo_ros_camera.so`）
Gazebo 深度相机插件，话题通过 remapping 对齐 ZED 真实驱动命名。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/zed2i/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | **彩色图像（YOLO 输入源）** |
| Pub | `/zed2i/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| Pub | `/zed2i/zed_node/depth/depth_registered` | `sensor_msgs/Image` | 深度图 |
| Pub | `/zed2i/zed_node/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |
| Pub | `/zed2i/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | 配准点云 |

> 分辨率：1280×720，帧率：30Hz，FOV：110°

#### 2.1.5 SBG IMU（URDF 中 `libgazebo_ros_imu_sensor.so` + `libgazebo_ros_sbg_driver.so`）

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/imu/data` | `sensor_msgs/Imu` | IMU 原始数据（50Hz） |
| Pub | `/imu/odometry` | `nav_msgs/Odometry` | SBG EKF 里程计（由 SBG 驱动发布） |

---

### 2.2 感知（Perception）

#### 2.2.1 ground_plane_segmenter_node（`ground_plane_segmenter.cpp`）
C++ 点云地面分割节点（PCL RANSAC）。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `velodyne_points` | `sensor_msgs/PointCloud2` | 原始点云 |
| Pub | `lidar/ground_plane` | `sensor_msgs/PointCloud2` | 地面点云 |
| Pub | `lidar/objects` | `sensor_msgs/PointCloud2` | 非地面物体点云 |

#### 2.2.2 lidar_detector_node（`node_lidar_detector.py`）
Python 锥桶检测节点（DBSCAN 聚类）。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/lidar/objects` | `sensor_msgs/PointCloud2` | 物体点云 |
| Pub | `/lidar/cone_detection` | `driverless_msgs/ConeDetectionStamped` | 检测到的锥桶 |
| Pub | `/lidar/cone_points` | `sensor_msgs/PointCloud2` | 锥桶点云（调试用） |

#### 2.2.3 gridmap_to_cone_detection_node（`node_gridmap_to_cone_detection.py`）
SLAM 栅格地图转锥桶检测节点。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/slam/occupancy_grid` | `nav_msgs/OccupancyGrid` | SLAM 栅格地图 |
| Pub | `/slam/cone_detection` | `driverless_msgs/ConeDetectionStamped` | 地图锥桶检测 |

#### 2.2.4 pointcloud_to_laserscan_node（第三方包）
点云转 2D 激光扫描。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/lidar/cone_points` | `sensor_msgs/PointCloud2` | remapped from cloud_in |
| Pub | `/lidar/converted_2D_scan` | `sensor_msgs/LaserScan` | remapped from scan |

#### 2.2.5 Velodyne 驱动节点（第三方包 `velodyne_driver` / `velodyne_pointcloud`）

| 节点 | 话题 | 消息类型 | 方向 |
|------|------|---------|------|
| velodyne_driver_node | `velodyne_packets` | `velodyne_msgs/VelodyneScan` | Pub |
| velodyne_transform_node | `velodyne_points` | `sensor_msgs/PointCloud2` | Pub |
| velodyne_laserscan_node | `/scan` | `sensor_msgs/LaserScan` | Pub |

---

### 2.3 规划（Planning）

#### 2.3.1 ft_planner_node（`node_ft_planner.py`）
FaSTTUBe 边界提取与路径规划器（基于 fsd_path_planning）。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `slam/cone_detection` | `driverless_msgs/ConeDetectionStamped` | 全局锥桶图 |
| Pub | `/planning/blue_bounds` | `nav_msgs/Path` | 蓝色边界 |
| Pub | `/planning/yellow_bounds` | `nav_msgs/Path` | 黄色边界 |
| Pub | `/planning/midline_path` | `nav_msgs/Path` | 中线规划路径 |

---

### 2.4 控制（Control）

#### 2.4.1 vel_to_ackermann_node（`node_vel_to_ackermann.py`）
Twist 速度指令转 Ackermann 控制指令。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/control/nav_cmd_vel` | `geometry_msgs/Twist` | 导航速度指令 |
| Sub | `/planning/midline_path` | `nav_msgs/Path` | 中线路径（distance_ctrl=true 时） |
| Pub | `/control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | Ackermann 控制指令 |

#### 2.4.2 sine_controller_node（`node_sine.py`）
正弦波测试控制器。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 控制指令 |
| Pub | `/control/accel_command` | `ackermann_msgs/AckermannDriveStamped` | 加速度指令 |

---

### 2.5 导航（Nav2）

#### 2.5.1 Nav2 控制器/行为/BT 导航器（`navigation.launch.py`）
通过 remapping 接入自定义话题。

| Remap 源 | Remap 目标 | 说明 |
|---------|-----------|------|
| `cmd_vel` | `control/nav_cmd_vel` | Nav2 输出速度指令 |
| `plan` | `planning/midline_path` | Nav2 输入路径 |
| `map` | `slam/occupancy_grid` | Nav2 输入地图 |

#### 2.5.2 odom_filter_node（`robot_localization` EKF）
扩展卡尔曼滤波定位。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `imu/odometry` | `nav_msgs/Odometry` | IMU 里程计 |
| Sub | `vehicle/wheel_twist` | `geometry_msgs/TwistWithCovarianceStamped` | 轮速 twist |
| Sub | `control/nav_cmd_vel` | `geometry_msgs/Twist` | 控制输入 |
| Pub | `odometry/filtered` | `nav_msgs/Odometry` | EKF 滤波后里程计 |

#### 2.5.3 slam_toolbox_node（`slam_toolbox`）
SLAM 建图。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Pub | `/slam/graph_visualisation` | — | 图可视化 |
| Pub | `/slam/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 位姿 |
| Pub | `/slam/occupancy_grid` | `nav_msgs/OccupancyGrid` | 栅格地图 |

---

### 2.6 系统状态与任务管理

#### 2.6.1 vehicle_supervisor_node（`node_vehicle_supervisor.py`）
车辆监督节点，管理系统状态与任务启动。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `imu/odometry` | `nav_msgs/Odometry` | IMU 里程计 |
| Sub | `lidar/cone_detection` | `driverless_msgs/ConeDetectionStamped` | LiDAR 锥桶检测 |
| Sub | `slam/cone_detection` | `driverless_msgs/ConeDetectionStamped` | SLAM 锥桶检测 |
| Sub | `system/shutdown` | `driverless_msgs/Shutdown` | 关机/EBS 信号 |
| Sub | `system/laps_completed` | `std_msgs/UInt8` | 完成圈数 |
| Sub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息 |
| Pub | `system/av_state` | `driverless_msgs/AVStateStamped` | 自主状态 |
| Pub | `system/ros_state` | `driverless_msgs/ROSStateStamped` | ROS 系统状态 |
| Srv(Client) | `launch/inspection`, `launch/ebs`, `launch/trackdrive`, `launch/system` | `std_srvs/SetBool` | 任务启动请求 |

#### 2.6.2 keyboard_controller_node（`node_controller.py`）
终端键盘控制器。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/system/as_status` | `driverless_msgs/State` | AS 状态 |
| Pub | `/control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 手动控制指令 |
| Pub | `/system/mission_select` | `std_msgs/UInt8` | 任务选择 |
| Pub | `/system/reset` | `driverless_msgs/Reset` | 重置 |
| Pub | `/system/r2d` | `std_msgs/Bool` | Ready-to-Drive |
| Pub | `/system/laps_completed` | `std_msgs/UInt8` | 圈数 |

#### 2.6.3 as_state_controller_node（`node_state_controller.py`）
AS 状态控制器。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `on_request/lap`, `on_request/mission`, `on_request/state` | `std_msgs/UInt8` | 状态请求 |
| Pub | `system/as_status` | `driverless_msgs/State` | AS 状态 |

---

### 2.7 硬件接口（实车）

#### 2.7.1 velocity_controller_node（`component_velocity_controller.cpp`）
速度 PID 控制器。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `system/av_state` | `driverless_msgs/AVStateStamped` | AV 状态 |
| Sub | `system/ros_state` | `driverless_msgs/ROSStateStamped` | ROS 状态 |
| Sub | `control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 控制指令 |
| Sub | `imu/velocity` | `geometry_msgs/TwistStamped` | IMU 速度 |
| Pub | `can/canbus_carbound` | `driverless_msgs/Can` | CAN 输出（扭矩请求） |
| Pub | `vehicle/combined_velocity` | `std_msgs/Float32` | 合成速度 |
| Pub | `vehicle/torque_request` | `std_msgs/Float32` | 扭矩请求 |

#### 2.7.2 steering_actuator_node（`component_steering_actuator.cpp`）
转向执行器（C5E 步进电机）。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `can/canopen_rosbound` | `driverless_msgs/Can` | CANopen 输入 |
| Sub | `vehicle/steering_angle` | `std_msgs/Float32` | 转向角反馈 |
| Sub | `control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 控制指令 |
| Sub | `system/av_state` | `driverless_msgs/AVStateStamped` | AV 状态 |
| Sub | `system/ros_state` | `driverless_msgs/ROSStateStamped` | ROS 状态 |
| Pub | `can/canbus_carbound` | `driverless_msgs/Can` | CAN 输出 |
| Pub | `vehicle/encoder_reading` | `std_msgs/Int32` | 编码器读数 |
| Pub | `vehicle/stepper_request` | `std_msgs/Int32` | 步进目标 |

#### 2.7.3 canbus_translator_node（`component_canbus_translator.cpp`）
CAN 总线翻译节点。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `/can/canbus_carbound` | `driverless_msgs/Can` | ROS→CAN |
| Pub | `/can/canbus_rosbound` | `driverless_msgs/Can` | CAN→ROS |
| Pub | `/can/canopen_rosbound` | `driverless_msgs/Can` | CANopen→ROS |
| Pub | `/vehicle/steering_angle` | `std_msgs/Float32` | 转向角 |
| Pub | `/vehicle/wheel_speed1` ~ `/vehicle/wheel_speed4` | `std_msgs/Float32` | 四轮速度 |
| Pub | `/vehicle/wheel_twist` | `geometry_msgs/TwistWithCovarianceStamped` | 轮速 twist |

---

### 2.8 可视化

#### 2.8.1 display_node（`node_display.py`）
锥桶检测可视化节点。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|---------|------|
| Sub | `lidar/cone_detection` | `driverless_msgs/ConeDetectionStamped` | LiDAR 锥桶 |
| Sub | `slam/cone_detection` | `driverless_msgs/ConeDetectionStamped` | SLAM 锥桶 |
| Sub | `control/driving_command` | `ackermann_msgs/AckermannDriveStamped` | 控制指令 |
| Sub | `lookahead_point` | `geometry_msgs/PointStamped` | 预瞄点 |
| Pub | `debug_markers/lidar_markers` | `visualization_msgs/MarkerArray` | LiDAR 锥桶标记 |
| Pub | `debug_markers/slam_markers` | `visualization_msgs/MarkerArray` | SLAM 锥桶标记 |
| Pub | `debug_markers/lookahead_marker` | `visualization_msgs/Marker` | 预瞄点标记 |

---

## 3. YOLO 视觉接入参考

### 3.1 当前视觉相关话题

| 话题 | 消息类型 | 当前来源 | YOLO 接入用途 |
|------|---------|---------|--------------|
| `/zed2i/zed_node/rgb/image_rect_color` | `sensor_msgs/Image` | Gazebo 相机/ZED 驱动 | **YOLO 图像输入** |
| `/zed2i/zed_node/rgb/camera_info` | `sensor_msgs/CameraInfo` | Gazebo 相机/ZED 驱动 | 相机内参（投影用） |
| `/zed2i/zed_node/depth/depth_registered` | `sensor_msgs/Image` | Gazebo 相机 | 深度图（锥桶距离估计） |
| `/zed2i/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | Gazebo 相机 | 3D 点云 |
| `vision/cone_detection` | `driverless_msgs/ConeDetectionStamped` | ConeDetectionPlugin（仿真） | **YOLO 输出目标话题** |

### 3.2 建议的 YOLO 节点设计

```
YOLO 检测节点（建议命名：yolo_cone_detection_node）
├── Sub: /zed2i/zed_node/rgb/image_rect_color  (sensor_msgs/Image)
├── Sub: /zed2i/zed_node/rgb/camera_info       (sensor_msgs/CameraInfo)
├── [可选] Sub: /zed2i/zed_node/depth/depth_registered  (sensor_msgs/Image)
└── Pub: vision/cone_detection                 (driverless_msgs/ConeDetectionStamped)
```

### 3.3 锥桶颜色定义（`driverless_msgs/Cone.msg`）

```
uint8 BLUE=0
uint8 YELLOW=1
uint8 ORANGE_SMALL=2
uint8 ORANGE_BIG=3
uint8 UNKNOWN=4
geometry_msgs/Point location
uint8 color
```

### 3.4 下游消费方

`vision/cone_detection` 当前**没有**被 Planner 直接订阅。Planner 订阅的是 `slam/cone_detection`。
若要让 YOLO 检测结果参与规划，有两种方案：

1. **融合方案**：新建一个融合节点，将 `lidar/cone_detection` 与 `vision/cone_detection` 融合后发布到 `slam/cone_detection`
2. **直接替换方案**：将 Planner 的订阅话题从 `slam/cone_detection` 改为 `vision/cone_detection`（不推荐，丢失 SLAM 全局信息）

---

## 4. 自定义消息类型速查

### driverless_msgs/ConeDetectionStamped
```
std_msgs/Header header
Cone[] cones
ConeWithCovariance[] cones_with_cov
```

### driverless_msgs/Cone
```
uint8 BLUE=0
uint8 YELLOW=1
uint8 ORANGE_SMALL=2
uint8 ORANGE_BIG=3
uint8 UNKNOWN=4
geometry_msgs/Point location
uint8 color
```

### driverless_msgs/AVStateStamped
```
std_msgs/Header header
uint8 mode          # MANUAL=0, AUTONOMOUS=1
uint8 mission       # NONE=0, MANUAL_DRIVING=1, INSPECTION=2, EBS_TEST=3, TRACKDRIVE=4
uint8 state         # OFF=0, READY=1, DRIVING=2, EMERGENCY=3, END=4
```

### driverless_msgs/ROSStateStamped
```
std_msgs/Header header
bool steering_ctrl
bool sbg_operational
bool lidar_operational
bool planning
bool good_to_go
bool finished
uint8 lap_count
uint16 identified_cones
uint16 mapped_cones
```

### driverless_msgs/State
```
uint8 state
uint8 mission
uint8 lap_count
```

### driverless_msgs/Shutdown
```
bool emergency_shutdown
bool finished_engage_ebs
```

### driverless_msgs/Reset
```
bool reset
```

### driverless_msgs/Can
```
uint32 id
uint8 dlc
uint8[] data
```

---

## 5. 关键 Launch 入口

| Launch 文件 | 启动内容 |
|------------|---------|
| `qutms_eufs_bridge/minimal_control_loop.launch.py` | eufs 仿真 + vel_to_ackermann 控制器（最小闭环） |
| `qutms_eufs_bridge/eufs_control_sim.launch.py` | 仅 eufs 仿真 |
| `qutms_eufs_bridge/qutms_controller.launch.py` | 仅 vel_to_ackermann 控制器 |
| `perception_bringup/perception.launch.py` | Velodyne 驱动 + 地面分割 + LiDAR 检测 + pointcloud_to_laserscan |
| `vehicle_bringup/system.launch.py` | 显示 + URDF + 感知 + rosbag |
| `vehicle_bringup/mission.launch.py` | 任务 watcher + handler（Inspection/EBS/Trackdrive） |
| `nav_bringup/nav_stack_bringup.launch.py` | 定位（EKF + SLAM）+ Nav2 导航 |

### driverless_msgs/PathWithSpeeds（新增）
```
std_msgs/Header header            # stamp = 来源锥桶检测时间戳（E2E延迟测量）
geometry_msgs/PoseStamped[] poses # track 系路径点
float64[] target_speeds           # [m/s]，与 poses 等长
float64[] curvatures              # [1/m]，带符号（左转为正）
```

---

## 6. 升级链路节点（2026-10 新增：YOLO 主链 v2）

`yolo_autonomous.launch.py` 现启动以下五节点（原 path_follower + vel_to_ackermann 退出该链，代码保留）：

| 节点 | 包 | 订阅 | 发布 | 说明 |
|------|---|------|------|------|
| cone_mapper_node | cone_mapping | `slam/cone_detection` | `/mapping/cone_map`（ConeDetectionStamped）、`/mapping/cone_map_viz`（MarkerArray） | 观测计数加权合并建图：w=n/(n+1) 位置更新、颜色多数投票、min_observations 门槛、超龄淘汰 |
| ft_planner_node（升级） | planners | `/mapping/cone_map`（参数 cone_topic）、`/vehicle/velocity` | `/planning/path_with_speeds`（新）+ 原 `midline_path`/`blue_bounds`/`yellow_bounds` | 新增三点曲率 + 三趟速度规划（横加速度预瞄/前向加速/后向制动），参数见 ft_planner.yaml `speed_profile:` 段 |
| path_tracker_node | controllers | `/planning/path_with_speeds`、TF(track→base_link)、`/vehicle/velocity`、`/vehicle/steering_angle` | `/control/driving_command`（Ackermann，**转向单位为度**） | 自适应前视 PP（Ld=clamp(kv·v+kl)）+ 曲率前馈 + 滑移补偿（默认关）+ 转向 PID（抗饱和/微分先行/变化率限制）+ 速度斜坡；路径超时 0.5s 自行指令零速 |
| safety_watchdog_node | controllers | `/planning/path_with_speeds`、TF | `/control/driving_command` | **墙钟**超时（默认 2s，免疫 sim time 跳变）或 TF 丢失 → 10Hz 持续重发停车指令 |

已同步修复：`node_ft_planner.py` detection_callback 合并分支时间戳不刷新的 bug（原 `buf_time = now` 只改局部变量，导致持续观测的锥桶 15s 后被 Age 清出）。

### GUI 急停对接（2026-10 追加）
`qutms_test_gui` 的 EMERGENCY STOP 发布 `/system/shutdown`（emergency_shutdown=True）+ 单次零速指令。新链路中 `path_tracker_node` 与 `safety_watchdog_node` 均订阅 `/system/shutdown` 并**闭锁**（持续发零速，不受新鲜路径解锁），直至 `/system/reset` 清除——否则单次零速指令会在 50ms 内被 20Hz 控制循环覆盖。
