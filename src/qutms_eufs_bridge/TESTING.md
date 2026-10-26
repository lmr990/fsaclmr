# 最小控制闭环手动测试

`qutms_eufs_bridge` 不包含消息转换节点：QUTMS 与 eufs_sim 都使用
`/control/driving_command` 和 `ackermann_msgs/msg/AckermannDriveStamped`。

在已构建并 source 了 eufs_sim、QUTMS 和本包的 Linux ROS 2 环境中：

```bash
ros2 launch qutms_eufs_bridge minimal_control_loop.launch.py
```

先确认控制器的输出：

```bash
ros2 topic echo /control/driving_command
```

在另一个终端，以 10 Hz 发布低速直线命令；`0.5` 是 m/s：

```bash
ros2 topic pub --rate 10 /control/nav_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

再以更低速度验证左转；`angular.z` 是 rad/s：

```bash
ros2 topic pub --rate 10 /control/nav_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.1}}"
```

观察 `/vehicle/velocity`、`/vehicle/steering_angle`、`/odometry/sbg_ekf` 和 Gazebo 车辆。停止发布后，再明确发布零速命令。不要使用此启动包测试实车 CAN、感知、规划或任务状态机。
