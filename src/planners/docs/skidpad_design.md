# FSAC 八字绕环（Skidpad）专用模式 — 设计报告

> 分支 `skidpad-mode`，HEAD `bcc90e8` 基线上的增量设计。
> 配套文档：[验收报告](skidpad_acceptance.md)、[RViz 截图](skidpad_rviz.png)、[planners/README](../README.md)。
> 代码位置：`src/planners/planners/skidpad_*.py`、`src/planners/config/skidpad.yaml`、`src/vehicle_bringup/`（任务层）、`src/qutms_test_gui/`（一键启动）。

---

## 1. 背景与问题定义

### 1.1 FSAC 规则约束（2025 规则 §6.4/§6.6）

| 项 | 值 |
|---|---|
| 两圆心距 | 18.25 m |
| 内/外圆直径 | 15.25 m / 21.25 m（赛道宽 3 m ⇒ 中心线半径 **R = 9.125 m**） |
| 程序（写死） | 垂直驶入 → 右圆 1 圈（不计时）→ 右圆 1 圈（计时）→ 左圆 1 圈（不计时）→ 左圆 1 圈（计时）→ 同向驶出 |
| 左/右定义 | 相对**入场车头方向**，不是地图方位 |
| 交叉点几何 | 两圆在交叉点相切，切线方向 = 入场方向 |

### 1.2 为什么现有规划器不能跑八字（结构性原因，非调参可解）

| 规划器 | 失效机理（代码级） |
|---|---|
| urinay | 中心线必须自交，但 `WayComputer` 候选边过滤规则 7 与 `Failsafe.cpp` 硬编码禁止轨迹自交（`allow_intersection: false`）；环路闭合检测（`max_dist_loop_closure: 1.0 m`）会把单圈误判为整圈闭合。实测输出：蓝/黄边界 + 绿色中线**只沿中央直道穿过交叉点，两圆被完全忽略** |
| ft_planner | 左右边界配对（ConeMatching）在两圆锥桶交错的交叉点失效；实测同样只沿入场直道出线 |
| 任务层 | `vehicle_bringup` 只有 trackdrive / inspection / ebs_test 三个 handler；GUI 的 skidpad 赛道条目无后端 |

结论：八字必须走**专用模式**——模板拟合 + 解析生成 + 按圈数状态机推进。这是 BIT（解析 `genTraj` + 模板匹配）与 fs-feup（模板 ICP）的共同路线；两个仓库为 GPL-3.0，本项目**只借鉴方法、独立实现**。

---

## 2. 总体架构

### 2.1 数据流

```
                        ┌────────────────────────────────────────────┐
 /mapping/cone_map ────▶│            skidpad_planner_node            │
 (ConeDetectionStamped) │                                            │
                        │  ① 预处理    select_boundary_cones          │
 TF track ← base_link ─▶│  ② 模板拟合  skidpad_fitting  (质量门+冻结)  │
 (车位姿/入场航向)       │  ③ 解析路径  skidpad_geometry (自检)        │──▶ /planning/path_with_speeds
                        │  ④ 速度      speed_profile (复用)           │    （18 m 窗口, 10 Hz）
 /system/shutdown ────▶│  ⑤ 进度      skidpad_progress (单调)        │──▶ planning/skidpad_state  (JSON)
 /system/reset ───────▶│  ⑥ 窗口发布  只发进度前方切片               │──▶ planning/skidpad_full_path (latched)
                        └────────────────────────────────────────────┘──▶ planning/skidpad_markers (RViz)
                                                                              │
                        ┌────────────────────────────────────────────┐        │
                        │ controllers/path_tracker_node（零修改）     │◀───────┘
                        │ 全局最近点 + Pure Pursuit + PID → 阿克曼    │
                        └────────────────────────────────────────────┘
```

**核心设计约束：`/planning/path_with_speeds` 在任何时刻只允许一个发布者**（skidpad 模式下 urinay/ft 链路被 launch 条件禁用），跟踪器与看门狗完全复用，`control.yaml` 未改动。

### 2.2 模块与职责

| 模块 | 纯函数? | 职责 |
|---|---|---|
| `skidpad_fitting.py` | 是（仅 scipy/numpy） | 蓝/黄边界锥 → 双圆模板（圆心×2、r_in、r_out）+ 质量门 + 鲁棒剔除 |
| `skidpad_geometry.py` | 是 | 模板 → 六段解析路径 + 曲率/速度 + 几何自检 |
| `skidpad_progress.py` | 是 | 单调弧长进度 + 发布窗口索引 + 离径/完成状态机 |
| `node_skidpad_planner.py` | ROS 节点 | 串起 ①–⑥：拟合重试（5 Hz）、冻结（连续 5 次漂移 <0.15 m）、TF 位姿、窗口发布、状态 JSON、shutdown/reset |
| `config/skidpad.yaml` | — | 全部参数（禁止散落硬编码） |
| `vehicle_bringup/node_skidpad_handler.py` | ROS 节点 | 与 trackdrive handler 同构：AV/ROS 状态、debug 模式；订阅 `planning/skidpad_state` 计 1–4 圈，FINISHED → shutdown |
| `vehicle_bringup/mission_launch/skidpad.launch.py` | launch | 实车任务：ZED 管线 + skidpad planner + 跟踪器（无 Nav2） |
| `qutms_eufs_bridge/yolo_autonomous_loop.launch.py` | launch | 仿真一键入口；**守卫**：`track:=skidpad` ⇒ 强制 `planner:=skidpad` 且默认关前馈 |
| `planners/skidpad_yolo_warmup.py` | CLI/GUI 助手 | 仿真冷启动热身：GT 圈建图 → 复位 → 交还 production planner |
| `qutms_test_gui`（main_window） | GUI | skidpad 赛道 + Start Simulator = 一键（仿真 RUNNING 后自动拉起热身助手） |

### 2.3 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 入 | `/mapping/cone_map` | `ConeDetectionStamped` | 拟合输入（生产=YOLO 融合图；仿真热身助手用 GT 图） |
| 入 | TF `track ← base_link` | — | 入场位姿/航向（可用参数 `entry_heading_deg` 覆盖）+ 进度反馈 |
| 入 | `/system/shutdown`、`/system/reset` | `Shutdown`/`Reset` | 与全栈一致的急停/复位语义 |
| 出 | `/planning/path_with_speeds` | `PathWithSpeeds` | **窗口切片**（poses/target_speeds/curvatures 等长同步切片），10 Hz |
| 出 | `planning/skidpad_state` | `std_msgs/String` | JSON：state/segment/lap/s_prog/拟合指标/未过门原因 |
| 出 | `planning/skidpad_full_path` | `nav_msgs/Path` | 完整八字（transient_local，供 RViz 验收） |
| 出 | `planning/skidpad_markers` | `MarkerArray` | 路径线 + 两圆心 + 交叉点 |

---

## 3. 各环节算法

### 3.1 预处理：颜色过滤

`select_boundary_cones` 只保留蓝（0）/黄（4）锥。

- **为什么过滤**：EUFS skidpad 的发车线是 24 个橙桶，几何上散布在入场直道两侧但**不在圆环上**；不过滤会被 20% 离群上限直接拒掉整个拟合（实测：含橙桶 84 锥 → "outliers 59/84 exceed 20%"）。
- **为什么不影响"颜色无关"优势**：过滤只是剔除发车线，圆环拟合本身不看颜色；蓝/黄误分类互换对几何零影响。

### 3.2 左右圆分组：法向投影 + 1D 二均值

入场航向单位向量 `e`（TF 航向或参数覆盖），其右手侧法线 `n = right(e) = (e_y, −e_x)`。所有边界锥投影 `p = c · n`，做一维两均值（k-means k=2，30 次迭代）分成两组。

**实现取舍**：最初用"最大间隙"切分，在合成数据上失败——相邻锥的投影间距（内环锥角距 ≈ 2π·7.6/15 ≈ 3.2 m）可以**大于**两圆的投影间隙，最大间隙不稳定。二均值对投影分布的双峰性鲁棒。入口直道的锥在两组里各占一半，由后续残差剔除消化。

### 3.3 共圆心双圆拟合（核心）

物理模型：每个圆的内/外两排锥**共圆心**、半径 `r_in / r_out`。

1. **初值**：Kasa 代数圆拟合（线性最小二乘）得圆心；到圆心距离做 1D 二均值得 `r_in / r_out` 初值。
2. **非线性精化**：`scipy.optimize.least_squares`，参数 `θ = (cx, cy, r_in, r_out)`，残差
   `r_i(θ) = ‖c_i − (cx,cy)‖ − r_assigned(i)`，
   `r_assigned` 按当前内/外环归属取 `r_in` 或 `r_out`；`loss="soft_l1", f_scale=0.1`（抑制幽灵锥影响），有界（半径 ∈ [0.1, 50]）。
3. **归属迭代**：拟合后按"距哪环更近"重新分配每个锥的内/外归属，再拟合；最多 8 轮或收敛（归属不变且参数变化 <1e-4）。退化（全分到一侧）时按距离中位数强制对半。
4. **鲁棒重拟合（两轮）**：合并两组残差，`residual > 0.6 m` 记为离群剔除；剔除比例 >20% ⇒ **拟合失败**（防幽灵锥淹没有效约束，实测 YOLO 幽灵率 ~37% 的前提）；只保留内点再重拟合一次。被剔除数量与最终 RMS 必须进日志。

### 3.4 质量门（全部满足才冻结，任何一项不过不发布路径）

| 门 | 阈值 | 防的失效 |
|---|---|---|
| 内点总数 / 每圆 | ≥40 / ≥16 | 部分圆被误当噪声剔光 |
| RMS / p90（内点） | ≤0.35 m / ≤0.5 m | 幽灵锥污染后的"过拟合" |
| 圆心距 | [16.0, 20.5] m | 把一条直道拟成"两个小圆" |
| R | [7.5, 11.0] m | 环宽/半径比例异常 |
| 环宽 `r_out−r_in` | [2.4, 3.6] m | 内外环被合并成单圆 |
| `|圆心距/2 − R|` | ≤0.5 m | 两圆不相切（模板不自洽） |
| 两圆 R 之差 | ≤0.5 m | 两组各自拟到了不同尺寸的环 |
| 拟合轴 vs 入场法线夹角 | ≤15°（仅告警） | 分组/航向错误 |

冻结条件：连续 **5 次**拟合成功且圆心漂移 **<0.15 m**；冻结后行驶中**不再重拟合**（防行驶中地图边缘变化拉歪模板），只有 `/system/reset` 解冻。

**实测精度**：GT 图拟合圆心误差 ≈0（构造即真值）；一轮 YOLO 热身后圆心误差 0.02–0.05 m、RMS 0.02–0.08 m、剔除 0–7 个。

### 3.5 左右圆判定与解析路径生成

- 交叉点 `o` = 两圆心连线中点；右圆 = 满足 `cross(e, c−o) < 0` 的那个（z 向上二维叉积）——**在入场系中定义，禁止按地图方位写死**。
- **关键归一化**：拟合圆心只用来提供 `o` 和 `R`；生成时把两圆心重造在 `o ± R·right(e)`。这保证①两圆在 `o` 精确相切；②交叉点公切线精确等于 `e`；③顺带消除拟合的微小旋转（旋转量以 `alignment_deg` 报告，>15° 告警）。
- 六段路径 `ENTRY → RIGHT_1 → RIGHT_2 → LEFT_1 → LEFT_2 → EXIT`：
  - `ds = 0.25 m` 等弧长采样；直线按弧长取点，圆弧按**精确等分角**（`n = ceil(2πR/ds)`，一步 `2π/n`，整圈无累计误差）；
  - 段边界不重复点（每段含起点不含终点，EXIT 例外含终点）——交叉点在整条路径中出现 5 次，这是刻意的（车真的经过 5 次）；
  - **绕向由断言决定而非手写符号**：每个圆在交叉点的起始角取 `o − c` 的方位角，绕向取使"首弦方向 ≈ e"（点积 >0.999）的那个；生成后断言，违背即抛错不发布。期望结果（入场朝北）：右圆顺时针、曲率 ≈ −1/R；左圆逆时针 ≈ +1/R（与 `PathWithSpeeds` 左正右负约定一致）。
- **生成自检**（失败不发布并报原因）：总长 = entry + 4·2πR + exit 误差 ≤2%；相邻点间距 ∈ (0, 1.5·ds]；航向跳变 ≤2°（含所有拼接处与交叉点）；各圆弧长 = 2πR ±1%；各点到所属段理想几何偏差 ≤0.05 m；刚体不变性由单测覆盖（入场角 0°/37°/180°/270° 全重验）。

### 3.6 速度规划（复用库，不另起炉灶）

- 曲率：`speed_profile.compute_curvatures`（三点外接圆公式 + 滑动平均）。
- 速度：`compute_speed_profile` 三遍法——①横向加速度上限 `v = √(a_lat/|κ|)` 带 5 m 前瞻（提前刹车）；②前向加速限制；③后向制动限制。仅覆盖 `speed_cap`（**测试值 1.2 m/s**，注释标明非车辆极限）。
- **附加刹停**：库的 `decel_to_stop_at_end` 只把最后一点压到 `min_speed`；八字要求 EXIT 末端真正停车，因此在其上再叠加一次全局后向制动 pass，把末段 `stop_len=4 m` 内平滑收到 0（单测断言最后 6 点单调不增）。

### 3.7 单调进度跟踪 + 窗口化发布（本设计的关键）

**问题**：现有跟踪器用**全局最近点**（`np.argmin`）定位，而八字中心线 5 次经过交叉点——全局最近点在交叉点必然歧义，目标点会跳回早期分段。

**方案**：路径不发全量，只发进度前方的窗口；进度本身用局部搜索保持单调。

1. 初始化：任务开始车必在 ENTRY，只在 ENTRY 段内找最近点定 `s_prog`。
2. 每周期（10 Hz）：只在弧长窗 `[s_prog − 1, s_prog + 8]` 内找最近点；`s_prog ← max(s_prog, 候选)`（含 0.1 m 噪声容差的隐性防回退）——**永不回退，交叉点不跳段**；只有 `/system/reset` 清零。
3. 窗口：`[s_prog, s_prog + 18 m]`（> 跟踪器最大前视 8 m + 余量），切片同步携带 speeds/curvatures；窗口内至多含一次交叉点 → 跟踪器的全局最近点在窗口内唯一。
4. TF 取不到：不更新进度、不发新窗口（靠跟踪器 0.5 s 超时停车兜底）。
5. 离径故障：车到路径距离（进度窗内量）>2.0 m 持续 1 s → `FAULT`，停发，等 reset（不做自动找回）。
6. 完成：`s_prog ≥ 总长 − 3 m` → `FINISHED` 停发。**偏差说明**：规范写 0.5 m，但未改动的跟踪器最小前视 2.0 m——其目标速度点在几何终点前 ~2 m 即为 0，车物理上停在那里；3 m 是实测标定（首轮 2.0 m/s 测试停在 262.4/265.4 处验证）。
7. 状态 JSON 与路径同频发布（`segment/lap/s_prog/state/拟合指标`），lap 编号固定：ENTRY=0、RIGHT_1/2=1/2、LEFT_1/2=3/4、EXIT=5。

### 3.8 冷启动热身（仿真专用，`skidpad_yolo_warmup`）

**为什么必须**：40/16 的门限 vs 仿真相机 10 m/110° 的视场——静止可见 13–26 个边界锥；即使沿入场直道开完全程，FOV 并集也只有 22–26 个（两圆的大部分始终在前进视场的侧/后方）。冷启动**几何上不可能**凑够门限（ sensor 覆盖事实，非拟合 bug；规范禁止放宽门限，故按 §0.3 报告并以热身流程解决）。

流程（单命令，GUI 一键自动调用）：
```
0) 停掉现有 skidpad planner（保证单发布者；pgrep 轮询确认无孤儿进程）
1) 生成临时 planner（输入=GT 图）→ 驱车跑八字，同时 YOLO/cone_mapper 积累真实地图
2) GT 圈到 LEFT_1（地图 ~60 边界锥）→ 停临时 planner
3) /system/reset_car_pos 复位 + TF 校验到起点（|x|,|y|<0.3, |yaw|<0.1 rad，最多重试 5 次）
4) 生成 production planner（输入=/mapping/cone_map）→ 在起点拟合冻结 → 正式跑
```
实现中处理过的竞态（都在实测中暴露）：`ros2 run` 包装进程退出 ≠ 子进程退出（孤儿的 warm planner 会把复位当成离径 FAULT）→ pkill+轮询；复位瞬间被残留窗口拖走 → TF 校验重试；成功退出时不得清理 production planner（`rcl_shutdown` 连坐崩溃）→ 成功路径无清理。

### 3.9 启动守卫与一键化

- launch 层守卫：`track:=skidpad` ⇒ `planner:=skidpad`（LogInfo 告警），`path_tracker_ff_gain` 默认 1.0 时自动置 0。**理由（实测）**：纯跟踪项已含路径曲率，再叠加跟踪器的曲率前馈在 R=9.1 恒定圆上双重计入 → 两个圆各稳定跑宽 ~0.7 m；关掉后圆上横偏 RMS 从 0.69 m 降到 0.22 m。`control.yaml` 未动，其他模式不受影响。
- GUI：skidpad 赛道自动选 skidpad 规划器；仿真 RUNNING 后自动启动热身助手；Stop/关窗清扫助手与遗留 planner。

---

## 4. 验收结果摘要（详见验收报告）

EUFS `skidpad` 赛道，`speed_cap=1.2 m/s`（测试值）、`ff_gain=0`，两轮均完整跑完 4 圈并 FINISHED：

| 指标（圆上） | 第 1 轮 GT 图 | 第 2 轮 YOLO 图 | 门槛 |
|---|---|---|---|
| 横向误差 RMS / max | 0.220 / 0.263 m | 0.219 / 0.270 m | ≤0.35 / ≤0.8 m |
| 分段顺序 | ENTRY→R1→R2→L1→L2→EXIT，无回跳 | 同左 | 规则顺序 |
| 撞锥 | 0（最近中心距 1.14 m） | 0（1.09 m） | 0 |
| 路径发布间隔 max | 0.172 s | 0.124 s | <0.5 s |
| PathWithSpeeds 发布者 | 1 | 1 | 恒为 1 |
| 拟合 RMS / 圆心误差 | 0.0006 m / ≈0 | 0.035 m / ≤0.05 m | ≤0.35 m / ≤0.5 m |

单元测试 72/72 通过（含几何/拟合/进度共 16 个 skidpad 用例：分段弧长、切线断言、曲率符号、刚体不变性、含幽灵锥拟合、普通弯道拒绝用例、进度单调性、离径 FAULT、交叉点不跳段等）。

---

## 5. 已知问题与改进方向

### 5.1 感知与冷启动（优先级最高）

| 问题 | 现状/证据 | 改进方向 |
|---|---|---|
| 冷启动死锁 | 静止可见 13–26 锥 < 门限 40/16；沿入场并集也只有 22–26 | ① 实车实测 ZED+YOLO 有效量程（真实相机 >20 m，很可能冷启动即可过门）；② 仿真把 `camera_max_view_distance`/FOV 调到真实水平后重验；③ planner 加**引导直行 bootstrap**：先发入场直线（拟合前），边开边拟合，门过即切换——难点是交叉点位置在拟合前未知，可用固定长度 + 到达即停兜底 |
| YOLO 地图碎片化 | 同会话多次热身后 60 锥膨胀到 249 条目、47% 离群，撞 20% 剔除上限 | 拟合前对地图做 **1.0 m 预聚类**（取簇质心，先例：`cone_id_mapper.dedupe_radius=1.0`）；或 `cone_mapper` 按观测方差自适应合并 |
| Depth/RGB 时间戳错位 | 行驶中持续告警 0.2 s，检测位置随速度 smear | 检测端做深度-颜色帧硬同步（丢弃超窗对）或按自车速度补偿 |
| 起点橙桶依赖颜色过滤 | 若 YOLO 把橙桶误分成蓝/黄会污染分组 | 分组后按"到环距离"做一轮 RANSAC 式一致性校验，而非只信颜色 |

### 5.2 拟合与模板

| 问题 | 现状 | 改进方向 |
|---|---|---|
| 无 ICP 精配准 | v1 位姿完全来自共圆心最小二乘（Y OLO 图圆心误差 ≤0.05 m，够用） | v2 预留 `alignment` 接口：以解析模板为参考做 2D ICP，吸收不对称磨损/漏检造成的偏心 |
| 模板旋转只报告不校正 | `alignment_deg >15°` 仅告警；生成时以 `e` 重造圆心 | 若实车入场航向噪声大，可改为"以拟合轴为主、e 仅定左右" |
| 冻结后不重拟合 | 符合规范（防行驶中地图边缘拉歪模板） | 若实车出现车体型变/颠簸导致系统性偏移，可加"受控重拟合"：仅当新拟合与冻结模板圆心差 <0.1 m 时静默更新 |

### 5.3 控制

| 问题 | 现状/证据 | 改进方向 |
|---|---|---|
| 圆上横偏 RMS ~0.22 m | 纯跟踪最小前视 2.0 m 在 R=9.1 上的固有切割 + 速度闭环超调（指令 1.2 实测峰值 1.6 m/s） | skidpad launch 传参下调 `min_lookahead`/提高 `steer_kp`（不改 control.yaml 默认）；或跟踪器支持按曲率缩放前视 |
| 速度低 | 1.2 m/s ⇒ 向心加速度仅 0.16 m/s²，远低于极限 | 横偏达标后按 `v = √(a_lat·R)` 逐级提 `speed_cap`（0.22 m 横偏下 3–4 m/s 应可行），计时圈再放开 |
| `finish_margin=3 m` 与规范 0.5 m 偏差 | 跟踪器最小前视 2 m 的物理停车点 | 长期：跟踪器支持"末端精确停车"（最后 3 m 内前视线性收缩到 0.5 m），则 margin 可回到规范值 |

### 5.4 安全与任务层

| 问题 | 现状 | 改进方向 |
|---|---|---|
| 离径阈值 2 m/1 s | 拍定测试值，未扫描调参 | 实车标定；可加"离径距离随速度缩放" |
| 进度卡死场景 | 跟踪器大幅落后（打滑/拥塞）时 s_prog 停滞 → 窗口停滞 → 车停（安全但非最优） | 可加"落后 >窗口长 → 降速/停车的显式状态" |
| 计时语义 | `laps_completed` 在 lap 1–4 均发布，未区分计时/非计时圈 | handler 按 `SEGMENT_LAP` 只在 2/4 圈发计时事件（对接计时系统时） |
| 任务类型占用 | supervisor 用 `skidpad_mission` 参数把 TRACKDRIVE 槽映射到 `launch/skidpad` | 长期在 `AVStateStamped`/DBC 增加 SKIDPAD mission 类型（涉嵌入式，需车队同步） |
| 实车验证 | 仅仿真验收；handler/`skidpad.launch.py` 未上硬件 | 推车测试：先 `perception:=false` + 手推验证 TF/窗口，再闭环 |

### 5.5 工程与工具链

| 问题 | 现状 | 改进方向 |
|---|---|---|
| 热身助手的仿真专用路径 | 依赖 `/system/reset_car_pos`、GT 话题 | 入口已按 `ros2 run planners skidpad_yolo_warmup` 固化；实车流程走 `skidpad.launch.py`，两者参数同源（skidpad.yaml） |
| GUI 无热身进度可视化 | 只有 Logs 页文本 + 状态栏 | 可在 Dashboard 加 skidpad 状态小部件（`planning/skidpad_state` 已含全部信息） |
| 状态 JSON 无 schema 校验 | 手写 json.dumps | 若接更多消费方，改 `std_msgs/String` → 自定义 `SkidpadState.msg` |
| 仿真 vs 实车时间源 | 节点用 `use_sim_time` 参数区分；助手脚本 `time.monotonic()`（墙钟） | 实车验证时确认进度/离径计时在墙钟下语义正确（rosbag 回放场景需重验） |

---

## 6. 参数与文件索引

- 参数全集：[`config/skidpad.yaml`](../config/skidpad.yaml)（几何/拟合门/进度/速度，注释含依据）。
- 测试：[`test/test_skidpad_geometry.py`](../test/test_skidpad_geometry.py)、[`test_skidpad_fitting.py`](../test/test_skidpad_fitting.py)、[`test_skidpad_progress.py`](../test/test_skidpad_progress.py)。
- 启动：仿真 `ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py track:=skidpad`（+ GUI 一键）；实车 `ros2 launch vehicle_bringup skidpad.launch.py`。
- 验收数据与统计脚本：[`docs/skidpad_acceptance.md`](skidpad_acceptance.md)、[`scripts/skidpad_acceptance_stats.py`](../scripts/skidpad_acceptance_stats.py)。

### 关键实测数字备忘

- 静止/沿途可见边界锥：13–26（门限 40/16）→ 冷启动必须热身（仿真）。
- `curvature_ff_gain` 1.0→0.0：圆上横偏 0.69 m → 0.22 m。
- `speed_cap` 2.0→1.2 m/s：横偏 0.53→0.22 m（叠加关前馈后）。
- 完成阈值 0.5→3 m：跟踪器最小前视 2 m 的物理停车点。
