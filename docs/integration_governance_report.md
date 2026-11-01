# 集成治理报告（integration governance report）

> Muse 集成治理单交付文档 · 执行日期 2026-11-02 · 执行环境：qutms_ws_1 工作副本（ROS2 Humble + Gazebo 可用，无相机硬件）
> 所有"已验证"结论附命令/输出位置；未亲测项明确标注**未验证**。

## 0. 纪律 1 — 基线漂移清单

任务锚点基于远端 main `074c38f`，执行前逐项核对结果：

| 项 | 074c38f | 工作副本 | 处置 |
|---|---|---|---|
| F2 相机 mount 默认值 | cam_z=0.697 / cam_pitch=0.0 | **0.9 / −0.21**（用户 2026-11-02 要求写入，未提交） | 先独立提交 `7ebc7dd`（pre-governance），再开始治理，保证各 Phase commit 纯净 |
| F11 锥高行号 | yaml:34 / node:113-114 | 一致（行号 ±2） | 按副本现状处理 |
| F6 localisation_params 行号 | 锚点 10,24 | 10 / 24 / 88 命中（debug 路径、base_footprint、磁偏角） | 按副本现状处理 |
| 其余 F1、F3-F5、F7-F12 | — | 一致 | — |

未提交文件 `GUI_START.md`（会话前已存在，与本单无关）保持未跟踪。

## 1. 各 Phase 完成项与 commit

| Phase | commit | 内容 | 验收状态 |
|---|---|---|---|
| 1 | `6ebb200` | `src/TOPICS_AND_NODES.md` 治理版重写 + `scripts/topic_inventory.py` 对账脚本 | 验收①对账见下；②待 Muse 抽查；③F1/F3/F10 条目齐（§6 frame 约定节） |
| 2 | `68fa356` | `src/system_smoke/` 独立包（tf/liveness/qos/params 四类检查，两 profile，非零退出码） | 三项注错全命中（§3）；两 profile 全链运行**未验证**（§4） |
| 3 | `89f6ebb` | profiles 三份 + wheelbase 1.580 单源 + pluginParams 相对路径 + F11 注释 + 冒烟接入 profiles | profile 被冒烟实际引用已验证；换机 clone 全 sim 启动**未验证**（§4） |
| 4 | `7136476` | LEGACY/UNUSED 标注（纯注释与 README，`git diff -U0` 自查零数值变更） | 完成 |
| 5 | 本 commit | `docs/frame_unification_plan.md` 草案（未执行任何改名） | 完成（仅文档） |

**Phase 1 对账**：脚本字面量 99 处调用点 → 去重 58 字符串 − 6 解析噪声 = 52 真实话题；参数派生解析后净增 7；前导斜杠等价合并 5 对 → **59 个真实话题**，文档 §1–§3 共 62 行覆盖（差额为 camera_info 双名/wheel_speed1..3 合并行），逐条说明在文档 §7。

**Phase 3 wheelbase 全仓口径**：`1.5`（作为 wheelbase）仅剩 TCone-YOLO-main vendor 镜像内副本（不参与构建）；`1.525` 为 `qutms_controller.launch.py:26` 显式注释对齐 `eufs_config/configDry.yaml` 车辆模型轴距的有据值，**未改动**并在此登记说明——如需一并单源化请用户裁决。

## 2. Phase 2 冒烟工具

`ros2 run system_smoke system_smoke --profile sim|real [--only tf,liveness,qos,params] [--window N] [--profile-file F]`

- 纯外部检查：TF buffer / 订阅计数与 frame 白名单 / `ros2 topic info -v` QoS 解析 / **节点参数服务直读**（rcl_interfaces/GetParameters，不经 ros2 CLI daemon——daemon 发现缓存对刚启动节点有竞态，已用原生 API 规避）。
- 四类检查均落地；profile 期望值由 `vehicle_bringup/config/profiles/`（phase 3）合并提供。

## 3. Phase 2 三项注错试验（本环境实测输出）

**注错 1 — cam_z +0.05 m（real 链 `perception:=false` + `cam_z:=0.95`）**
`/tmp/smoke_tf_badcamz.txt`：
```
FAIL [tf      ] mount base_link->zed2i_camera_link VALUE MISMATCH: z=+0.950 expect +0.900 (Δ0.050)
SUMMARY: 2 PASS / 1 FAIL / 0 SKIP -> SMOKE FAILED
```
**注错 2 — wheelbase 1.5（path_tracker_node 命令行注入 `-p wheelbase:=1.5`）**
`/tmp/smoke_wb_bad.txt`：
```
FAIL [params  ] /path_tracker_node wheelbase = 1.5 (expect 1.58)  <- MISMATCH (F4 class)
```
正常值（control.yaml 1.580）对照 `/tmp/smoke_wb_good.txt`：`2 PASS -> SMOKE PASSED`。
（注：实际注入采用节点参数注入而非改仓库文件——zero-file-change 注错，等价且可复现。）
**注错 3 — 停掉 cone_mapper（假检测流 5 Hz 喂 slam/cone_detection）**
mapper 在跑 `/tmp/smoke_live_ok.txt`：`PASS [liveness] /mapping/cone_map: 76 msgs, frame 'track'`；
杀掉后 `/tmp/smoke_live_bad.txt`：
```
FAIL [liveness] /mapping/cone_map: 0 messages in 8s
SUMMARY: 0 PASS / 1 FAIL / 0 SKIP -> SMOKE FAILED
```
real profile TF 正常配置对照 `/tmp/smoke_profile_merged.txt`：`3 PASS -> SMOKE PASSED`（含 profiles 合并期望值端到端）。单次冒烟运行时长 ≤ 30 s，满足 ≤3 分钟要求。**已验证**。

## 4. 未做项与原因

| 项 | 状态 | 原因 / 补做条件 |
|---|---|---|
| sim profile 全链冒烟（EUFS+YOLO 全 PASS 记录） | **未验证** | 需 Gazebo 有头/离屏渲染 + YOLO 权重 + GPU 的完整窗口；本环境仅验证了四类检查器的每一类与三项注错。补做：bench 窗口期 `ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py` + `system_smoke --profile sim`，前后各记录一份 |
| real profile 全链冒烟（含相机图像话题） | **未验证** | 无 ZED 硬件；TF/参数/QoS/建图活性类已在隔离夹具验证。补做：装车/上杆后同命令 |
| 换绝对路径 clone + 编译 + sim 启动（Phase 3 验收③） | **部分验证** | 解析函数单测通过（相对→本工作区 share 绝对路径，文件存在性断言通过）；完整 Gazebo 启动未验证（同上窗口） |
| Phase 2 验收②（Muse 抽查 10 条） | 待外部 | — |
| F5 GUI 重写行为（eufs_launcher GUI 使用时仍会把绝对路径写回 pluginParams.yaml） | 未改 | GUI 逻辑改动超出"路径写法"最小面；已注释警告 + 报告登记，建议后续单 |
| Phase 3 launch 参数结构改造（顶层 launch 读 profile 传参） | 未做（任务允许的退步） | 纪律 8：测试窗口期不动已跑通 launch 接线；profiles 已作为冒烟期望源落地 |

## 5. 执行过程中的新发现（F5 实锤，均未擅动）

1. **工作区双路径同体**：`/home/lmr/qutms_ws` 与 `/home/lmr/qutms_ws_1` 的 `build/`、`log/` 为**同一目录**（inode 相同），`install/setup.bash`、`local_setup.bash` 内硬编码 `/home/lmr/qutms_ws/...`。**建议**（登记不执行）：固定一份无空格、无后缀路径为唯一工作区，重建 install（`rm -rf build install && colcon build`）消除硬编码路径；shell 中严禁凭记忆 cd 工作区（本次执行中 shell 持久 cwd 曾漂回旧路径导致多次构建错位，已用显式 `cd` 规避）。
2. **旧 install 的 zed_msgs 与现 RMW 不匹配**：混用旧 `/home/lmr/qutms_ws/install` 的 zed_wrapper 时容器崩溃（`libzed_msgs__rosidl_typesupport_fastrtps_cpp.so` 缺失）。建议单一工作区重建时一并重建 zed 包。
3. **real wrapper v5 叶帧名与 sim URDF 不同**：`zed2i_left_camera_frame_optical`（实车）vs `zed2i_left_camera_optical_frame`（sim）——已分别写入两份 profile 的 `leaf_frames`；属于 F10 的又一别名证据，已补进 TOPICS_AND_NODES.md 的 frame 节（phase 1 文档 + 本报告）。
4. control.yaml 的 `wheelbase: 1.5` 属 `vel_to_ackermann_node`（非 tracker）；tracker 本就是 1.580。F4 的"并存"是两节点各一值，phase 3 已统一。

## 6. 测试对比（治理前/后同口径）

| 套件 | 治理前（074c38f） | 治理后（HEAD） | 结论 |
|---|---|---|---|
| cone_mapping/test | 28 passed（L1+L2 交付时基线） | **33 passed** | 无新增失败 |
| yolo_cone_detection/test | 7 passed | **7 passed** | 无新增失败 |
| planners/test | 需 ROS 环境补齐后运行 | **60 passed**（PYTHONPATH 含 driverless_msgs/driverless_common + LD_LIBRARY_PATH） | 无新增失败 |
| qutms_test_gui/test | 2 failed / 1 passed | 2 failed / 1 passed（相同用例） | **预存失败**（断言硬编码 /home/lmr/qutms_ws 路径，F5 类；该包治理中零文件改动，见 §5-1） |

**新增失败 = 0**。

## 7. 待用户确认项

1. **bench mount x/y/yaw 实测值**（`bench.yaml` 中为 null；launch 默认 cam_x=-0.370 仍是仿真 URDF 值——F2 的残留半项）。
2. car.yaml 全部 mount 量（整车装车后实测回填）。
3. `qutms_controller.launch.py` 的 wheelbase 1.525（对齐 configDry 车辆模型）是否纳入单源。
4. Phase 2/3 两个"未验证"补做的时间窗（sim 全链冒烟 + 换机 clone 验证）。
5. GUI 重写 pluginParams 绝对路径的行为是否在后续治理单中改为相对。
6. frame 改名（`docs/frame_unification_plan.md`）是否批准执行与时间窗。

## 8. 交付物索引

- `src/TOPICS_AND_NODES.md`（治理版）+ `scripts/topic_inventory.py`
- `src/system_smoke/`（含 config/sim.yaml、config/real.yaml）
- `src/vehicle_bringup/config/profiles/{sim,bench,car}.yaml`
- `docs/frame_unification_plan.md`（草案）
- 注错输出：docs/evidence/smoke_tf_badcamz.txt、/tmp/smoke_wb_bad.txt、/tmp/smoke_wb_good.txt、/tmp/smoke_live_ok.txt、/tmp/smoke_live_bad.txt、/tmp/smoke_profile_merged.txt（已归档至 docs/evidence/，可按 §3 命令复现）
