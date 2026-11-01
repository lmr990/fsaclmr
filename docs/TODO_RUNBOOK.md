# 待办与启动手册（Runbook）

> 2026-11-02 整理，对应 main `ac3a376`。所有命令都在 `/home/lmr/qutms_ws` 下执行；
> 终端需先 `source /opt/ros/humble/setup.bash`（日常终端若已加进 .bashrc 可省）。

## 0. 日常启动（现役 bench 链）

```bash
cd /home/lmr/qutms_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# 台架感知链（ZED 2i + 检测 + 建图；相机参数已按 2026-11-02 实测内置）
ros2 launch yolo_cone_detection zed_real_pipeline.launch.py

# 启动后跑一次冒烟（TF/活性/QoS/参数四类检查，任一 FAIL 退出码非零）
ros2 run system_smoke system_smoke --profile real

# 台架清幽灵（锥桶挪过位置后）
ros2 service call /cone_mapper/clear_map std_srvs/srv/Trigger

# RViz 看图：添加 /mapping/cone_map_viz（MarkerArray）；灰色小球=观察期 tentative
```

仿真链（EUFS）：

```bash
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py
ros2 run system_smoke system_smoke --profile sim
```

## 1. 马上要做（下次 bench/仿真窗口）

| # | 事项 | 怎么做 | 完成标准 |
|---|---|---|---|
| 1 | **遮挡回归四项**（L1+L2 幽灵修复的实测证据） | 手遮挡锥桶 1 s 后移开；人横穿画面；无遮挡基线 3 min；按 `docs/occlusion_ghost_acceptance.md` E 节手册执行 | 幽灵数=0、真桶偏移 ≤0.2 m、无跟随人的条目；数据回填该文档 E 节 |
| 2 | **sim 全链冒烟 before/after**（治理验收） | 起 `yolo_autonomous_loop.launch.py` 后跑 `system_smoke --profile sim`，正常配置一份 + 与治理前对照一份，输出存 `docs/evidence/` | 两份记录，after 全 PASS |
| 3 | **全量重建 install** | `cd /home/lmr/qutms_ws && rm -rf build install log && colcon build` | `source install/setup.bash` 后 `ros2 pkg prefix system_smoke` 能找到（修复后建包不被拾取） |
| 4 | **换路径 clone 验证**（Phase 3 验收③） | `git clone git@github.com:lmr990/fsaclmr.git /home/lmr/fsac_test && cd /home/lmr/fsac_test && colcon build && source install/setup.bash`，再启动仿真 | sim 正常启动、噪声/车辆参数生效（pluginParams 已相对化，自动解析到该 clone 的 share 目录）；验证完 `rm -rf /home/lmr/fsac_test` |
| 5 | **Muse 抽查**（治理验收②） | 外部执行：接口清单抽 10 条对代码、锚点抽查三处 | — |

## 2. 待用户决定（不阻塞任何事）

| # | 事项 | 建议 |
|---|---|---|
| 6 | `qutms_ws_old/`（752 MB，10-26 旧线） | `tar -czf /home/lmr/qutms_ws_old.tar.gz -C /home/lmr qutms_ws_old/src && rm -rf /home/lmr/qutms_ws_old`（先确认 src 无独有改动） |
| 7 | `qutms_ws.zip`（310 MB） | 你自己的备份，留删随意 |
| 8 | `qutms_controller.launch.py` wheelbase 1.525 | 对齐 configDry 车辆模型的有据值；如要单源化再说一声 |
| 9 | frame 改名收敛（base_footprint→base_link） | **仅批准后执行**，方案见 `docs/frame_unification_plan.md` |
| 10 | GUI 会把绝对路径写回 pluginParams.yaml | 后续治理单：GUI 改为写相对文件名 |

## 3. 造车之后（现在做不了）

| # | 事项 | 怎么做 |
|---|---|---|
| 11 | car.yaml 全部 mount 实测回填 | 装车后量相机相对 base_link 的 x/y/z/roll/pitch/yaw（pitch 误差对 (x,y) 是二阶，优先量准 x/y/yaw），填 `src/vehicle_bringup/config/profiles/car.yaml`，launch 参数同步改车值 |
| 12 | 实车定位链（F12 INS/CGI-410） | 独立任务，未开工 |
| 13 | `publish_track_tf:=false` | 定位接管 track→base_link 后，实车 launch 传该参数，冒烟 real profile 的恒等 TF 检查项同步调整 |

## 4. 快速参考

- **接口/TF/帧名**：`src/TOPICS_AND_NODES.md`（治理版；相机 optical 帧仿真/实车已统一为 `..._frame_optical`）
- **冒烟工具**：`ros2 run system_smoke system_smoke --profile sim|real [--only tf,liveness,qos,params]`
- **环境参数单源**：`src/vehicle_bringup/config/profiles/{sim,bench,car}.yaml`（冒烟自动读取合并）
- **治理报告**：`docs/integration_governance_report.md`（含未验证项清单）
- **幽灵修复验收**：`docs/occlusion_ghost_acceptance.md`（E 节待实测回填）
