# LEGACY — 上游 QUTMS 旧定位链（当前系统未使用）

> **status: LEGACY**（governance phase 4 标注，2026-11-02，基线 074c38f+）
> 本包是 QUTMS 上游的 nav2/robot_localization 定位链（slam_toolbox + EKF +
> base_footprint 命名）。当前系统的感知/规划链（yolo_autonomous、
> zed_real_pipeline、skidpad.launch）**不使用**本包；仅
> `vehicle_bringup/mission_launch/trackdrive.launch.py` 与
> `ebs_test.launch.py` 仍引用，属于两代架构并存（见该两文件头注释）。
>
> **启用前必须处理的三项遗产**：
> 1. **base_footprint 命名**：本链根帧为 `base_footprint`，与现链
>    `base_link` 不一致（F1 双名制，mission handler 查 `track→base_footprint`，
>    规划器查 `track→base_link`）；收敛方案见 `docs/frame_unification_plan.md`。
> 2. **磁偏角**：`config/localisation_params.yaml` 的
>    `magnetic_declination_radians: 0.1937` 为布里斯班（Brisbane）值，
>    换场地必须按当地磁偏角重算。
> 3. **调试绝对路径**：同文件 `debug_out_file: /home/ubuntu/QUTMS/tmp/...`
>    为旧车路径，换机即失效。

（原 QUTMS 上游 README 无内容，本说明为 governance phase 4 新增。）
