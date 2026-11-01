# frame 改名收敛方案（草案，未执行）

> governance phase 5 — 2026-11-02 起草，基线 `7136476`。
> **本文件只是方案草案：是否执行、何时执行由用户单独批准并指定时间窗。
> 在获得批准前，任何代码/参数不得按本文改动。**

## 1. 目标

消除 F1 双名制：独立感知/规划链用 `base_link`，整车 URDF 与旧 nav 链用
`base_footprint`，mission handler 与规划器各查各的根帧，同车混跑时 TF 断裂。
收敛方向**建议统一到 `base_link`**（理由：现役全部新链已用 base_link，
base_footprint 只存在于 vehicle_urdf 默认值与旧 nav 链配置；反向统一要改的
文件更多）。备选：维持双名但在 mission handler 处加 `base_footprint→base_link`
别名静态 TF（改动最小，见 §5 方案 B）。

## 2. 受影响文件清单（基于 src/TOPICS_AND_NODES.md 治理版逐项核对）

| # | 文件 | 现状 | 统一到 base_link 的动作 |
|---|---|---|---|
| 1 | `vehicle_urdf/urdf/vehicle.urdf.xacro` | 根 link `${base_frame}` | 无需改（参数化） |
| 2 | `vehicle_urdf/launch/robot_description.launch.py:81` | `base_frame` 默认 `base_footprint` | 默认值改 `base_link` |
| 3 | `vehicle_bringup/launch/system.launch.py:63` | `base_frame` 传 `base_footprint` | 改传 `base_link` |
| 4 | `vehicle_bringup/launch/push_log.launch.py` | 引用 base_footprint（待核） | 同步改 |
| 5 | `vehicle_bringup/node_*_handler.py`（mission handlers） | 查 `track→base_footprint` | 改查 `track→base_link` 或参数化默认值 |
| 6 | `nav_bringup/config/localisation_params.yaml:24` | `base_link_frame: base_footprint` | 改 `base_link`（LEGACY 包，启用才生效） |
| 7 | `nav_bringup` 其余 launch/params（ekf、slam_toolbox） | odom→base_footprint 链 | 同步改（LEGACY 包） |
| 8 | `eufs_config/config/pluginParams.yaml:5` | `base_frame: "base_link"` | 已是 base_link，不动 |
| 9 | `planners/node_ft_planner.py:156,351` | 默认 `base_link` | 不动 |
| 10 | `qutms_test_gui` 显示配置（rviz/话题过滤） | 含 frame 引用（待 grep 核对） | 逐处核对 |
| 11 | rviz 配置（`eufs_config/rviz/*.rviz` 等） | Fixed Frame/TF 过滤 | grep `base_footprint` 逐处替换 |

执行前用 `grep -rn "base_footprint" src/ --include="*.py" --include="*.yaml"
--include="*.rviz" --include="*.xacro"` 重新生成此表（以届时 HEAD 为准）。

## 3. 改名顺序（方案 A：统一 base_link）

1. **准备**：全仓 grep 生成受影响清单；选定时间窗（不得与 bench 实测重叠）。
2. **旧链隔离验证**：确认当前没有任何运行链依赖 base_footprint
   （system_smoke 两 profile 均 PASS 且 `ros2 run tf2_tools view_frames` 无
   base_footprint 帧）。
3. **vehicle_urdf 默认值 + system.launch 传参**（上表 2/3/4 项）一个 commit。
4. **mission handlers**（第 5 项）一个 commit。
5. **LEGACY nav_bringup 参数**（第 6/7 项）一个 commit（不影响现役链）。
6. **GUI/rviz 配置**（第 10/11 项）一个 commit。
7. 每步之后跑 §4 冒烟；全部完成后在 TOPICS_AND_NODES.md 的 F1 条目改为
   "已收敛"并注明 commit。

## 4. 每步验证点

- `system_smoke --profile real --only tf` 与 sim profile（Gazebo 窗口期）
- `ros2 launch vehicle_bringup launch/system.launch.py` 后
  `ros2 run tf2_tools view_frames`：根帧应为 base_link、无孤立树
- mission handler 启动日志无 "Transform exception"
- 单元测试全过（handlers 若有 TF 相关测试）

## 5. 回滚与备选

- **回滚**：每步独立 commit，`git revert` 即可；TF 静态无持久化状态。
- **方案 B（别名法，改动最小）**：保留 base_footprint，在 system 链加
  `base_footprint→base_link` 恒等静态 TF（一个 launch 参数）。缺点：双帧
  长期共存、违反"单一事实源"，仅作为时间窗不足时的过渡。

## 6. 预估风险

1. mission handler 与 supervisor 的字符串匹配（如按 frame 名查日志/服务）
   ——执行前 grep `base_footprint` 全仓（含 GUI）为准。
2. slam_toolbox/nav2 旧链参数文件里除 `base_link_frame` 外可能还有
   `odom_frame`/`map_frame` 联动假设——LEGACY 包启用前单独验证。
3. EUFS sim 插件 `pluginParams.yaml` 的 base_frame 必须与 URDF 根一致
   （已是 base_link，勿随改动误改）。
4. rviz 保存的视图/TF 前缀缓存（.rviz 内字符串）——逐文件替换后用
   `--fixed-frame` 冒烟。
5. 上游 QUTMS 同步：vehicle_urdf/nav_bringup 是上游代码，改名后与上游
   diff 扩大，回补上游修复时需人工对齐。

## 7. 不做清单（本草案边界）

不改代码、不建分支、不动 launch；本文件只回答"若要做，怎么做"。
