# 瞬态前景物幽灵修复（L1+L2）验收文档

`docs/occlusion_ghost_acceptance.md` — 按 Muse 验收清单 B/C/D/E 逐项回填。
**状态标注约定**：每项标注 ✅ 已实现并验证 / ⏳ 未测（附原因与执行手册）。
**诚实声明**：本环境为无相机、无 Gazebo 的代码工作区，E 节全部实测项未测，
只能交付单元测试与代码层证据；数值均来自测试断言与代码常量，无编造实测数据。

---

## A. 交付物

### A.1 改动文件清单（基线 HEAD：`4d6ac6c`，工作区未提交）

| 文件 | 改动（一句话） |
|---|---|
| `src/yolo_cone_detection/yolo_cone_detection/size_gate.py` | **新增** 纯函数 L1 门：按类别锥高带 [h_small, h_large] 做深度–尺寸双向校验，边缘豁免只豁矮不豁高 |
| `src/yolo_cone_detection/yolo_cone_detection/node_yolo_detector.py` | 深度采样/距离门之后、反投影之前接入 size_gate_reject；按 YOLO 类别解析锥高；新增 `size_gate_dropped` 独立计数进节流日志 |
| `src/yolo_cone_detection/config/yolo_detector.yaml` | 新增 `size_gate*` 三个参数与 `cone_large_height`，注明容差带为待标定先验 |
| `src/yolo_cone_detection/test/test_size_gate.py` | **新增** B 节单元测试 7 例 |
| `src/cone_mapping/cone_mapping/cone_mapper.py` | 重写纯逻辑：tentative 生命周期、innovation gate（σ 门 + 绝对 cap）、周期性冲突退役 `retire_superseded`、consolidate 尊重 gate |
| `src/cone_mapping/cone_mapping/node_cone_mapper.py` | 参数接线、回调改用 `process_detection`、publish 周期执行冲突退役、RViz 灰色 tentative 标记、dispositions 统计日志 |
| `src/cone_mapping/test/test_cone_mapper.py` | 重写 + 新增 C1/C2/C3 关键测试，共 33 例 |
| `src/yolo_cone_detection/launch/zed_real_pipeline.launch.py` | **bench 配置**：`min_observations` 3→5、`max_age_sec` 600→120（代码默认值未动，见 D） |
| `docs/occlusion_ghost_acceptance.md` | **新增** 本文档 |

### A.2 工作区状态

- HEAD：`4d6ac6c`（detector: base-point depth sampling, nearest-mode cluster, per-frame dedupe），分支 `main`。
- 以上改动全部为**未提交工作区改动**（`git status` 6 个 M + 2 个未跟踪源文件 + 本文档）。
- 会话开始前工作区已有 `node_cone_mapper.py` 的未提交修改（上一轮 L1+L2 初版）与未跟踪的 `GUI_START.md`（与本任务无关，未动）。
- **代码未推送**：按 F 节约定，本次验收结论仅到「文档与数据层面」，未做代码锚点抽查。

### A.3 参数总表

**新增参数**（代码默认值 = 交付值；bench 值仅在 launch/yaml 覆盖处单列）：

| 参数 | 所属节点 | 默认值 | 说明 |
|---|---|---|---|
| `size_gate` | yolo_detector | `true` | L1 门开关 |
| `size_gate_min_ratio` | yolo_detector | `0.55` | 矮框阈值（×预期高），**待标定** |
| `size_gate_max_ratio` | yolo_detector | `2.0` | 高框阈值（×预期高），**待标定** |
| `cone_large_height` | yolo_detector | `0.505` | 大橙锥高度 [m]，FSAC/FSG 规则值 |
| `tentative_max_age_sec` | cone_mapper | `5.0` | tentative 存活时间 [s]（C1） |
| `confirm_min_obs` | cone_mapper | `4` | 转正所需紧观测次数 K（C1） |
| `confirm_span_sec` | cone_mapper | `1.0` | 转正所需时间跨度 T [s]（C1） |
| `confirm_radius` | cone_mapper | `0.15` | 「紧观测」半径 [m]（C1） |
| `gate_sigma_factor` | cone_mapper | `2.5` | innovation 门 σ 系数（C2） |
| `gate_max_radius` | cone_mapper | `0.4` | innovation 门绝对上限 [m]，保护年轻条目（C2） |
| `conflict_radius` | cone_mapper | `1.5` | 冲突退役半径 [m]（C3） |
| `stale_conflict_sec` | cone_mapper | `5.0` | 条目「饥饿」判定时长 [s]（C3） |

**修改参数**（仅 bench/launch 配置，代码默认值不动）：

| 参数 | 代码默认值 | bench/launch 值 | 说明 |
|---|---|---|---|
| `min_observations` | `3`（未动） | `5`（zed_real_pipeline.launch.py） | 发布门限；理由见 D 节 |
| `max_age_sec` | `120.0`（未动） | `120.0`（launch 由 600 收紧） | established 驱逐时长；D 节有正式论证 |

---

## B. L1 检测端：深度–尺寸一致性门

**判定位置（衔接点）**：`_build_cones` 中，深度采样 + 最近簇 + `[min_range, max_range]` 距离门 + 深度无效丢框之后，反投影与 TF 变换之前（`node_yolo_detector.py` 搜索 `size_gate_reject`）。消息结构（`ConeDetectionStamped`）与协方差计算（`_range_std_for`）**未改动**。`depth_size_fallback`（单目尺寸反算距离）若开启，其 z 由框高推出、比值恒为 1，天然通过本门，无冲突。

### B-① 锥高按类别取值

| 类别（TCone-YOLO 类号） | h_small | h_large | 来源 |
|---|---|---|---|
| red/orange（0，图像无法分大小橙锥） | 0.305 m | **0.505 m** | FSAC/FSG 规则锥桶尺寸（小锥 0.305 m、大橙锥 0.505 m）；代码库原有 `cone_real_height: 0.305` 同源 |
| blue（1） | 0.305 m | 0.305 m（无大锥变体） | 同上 |
| yellow（2） | 0.305 m | 0.305 m | 同上 |

矮框按 h_small 判、高框按 h_large 判（橙类带宽 [0.55, 2.0]×[h_small, h_large] 两侧展开）。
**已知 slack（诚实交代）**：容差带是全套共享一对 (min_ratio, max_ratio)，蓝/黄类的上界等价于 2.0×0.305≈0.61 m 视高——0.5 m 视高的"蓝色大锥"误检不会被 B3 拒掉（0.61 m 内）。这类误检交给 L2 生命周期兜底；若 bench 实测需要，可给蓝/黄类单独收紧 `max_ratio`（预留参数级改动，未实现）。

### B-② 容差带数值与依据

- `min_ratio = 0.55`，`max_ratio = 2.0` — **先验值，标注：待标定**。
- 依据（先验推导）：YOLO 框通常略松（含底座反光/阴影，实测经验 1.1–1.3×）；下界 0.55 容纳框略紧 + 采样噪声，同时抓住"半锥可见"（比值 ~0.4–0.5）与"深度采到手"（比值 ≪0.3）；上界 2.0 保住 0.505 m 大橙锥（比值 1.66）同时拒掉站立人（比值 ≈5.6，距离无关）。
- **标定计划（待执行）**：bench 静态场景跑 5–10 min，在 `_build_cones` 中临时记录 `(cls, z, bbox_h, ratio)` 到 csv，取确认入图真锥的 ratio 分布 [p1, p99]，把 0.55/2.0 收紧到该分布外沿；标定后删除本「待标定」标注。

### B-③ 图像边缘豁免

框贴图像上/下边缘（`y1 <= 1 or y2 >= img_h - 2`）时跳过**矮框**判定（裁切天然变矮），**高框判定仍生效**（裁切只会变矮，超上界仍是真异常）。测试 `test_B4_edge_clipped_box_kept`、`test_B7_edge_clipped_tall_still_dropped`。

### B-④ 独立计数与日志

- `size_gate_dropped` 与深度无效丢弃 `no_depth_dropped` 分开计数（`node_yolo_detector.py`）。
- 汇总进现有的逐帧日志行，该日志 `throttle_duration_sec=2.0`，即每 2 s 一条，不逐框刷屏。

### B-⑤ 单元测试（`src/yolo_cone_detection/test/test_size_gate.py`，7 例全过）

| 测试名 | 清单条目 | 断言要点 |
|---|---|---|
| `test_B1_normal_cone_kept` | ① | 3 m/71 px、1.3 倍松框、8 m/30 px 均保留 |
| `test_B2_partial_occlusion_short_dropped` | ② | 手劫持深度（z=0.7 m/框 60 px）与半锥（z=5 m/框 20 px）均丢弃 |
| `test_B3_person_scale_tall_dropped` | ③ | 1.7 m 人 3 m/397 px、5 m/238 px（橙类带宽）均丢弃 |
| `test_B4_edge_clipped_box_kept` | ④ | 贴边矮框（3 m/30 px）保留 |
| `test_B5_orange_band_admits_large_cone` | 锥高按类别 | 0.505 m 大橙锥（118 px@3 m）橙类保留 |
| `test_B6_blueyellow_tighter_tall_bound` | 锥高按类别 | 0.7 m 物体@3 m：蓝黄拒、橙类留 |
| `test_B7_edge_clipped_tall_still_dropped` | 豁免边界 | 贴边但超高的框仍丢弃 |

通过记录：`pytest src/yolo_cone_detection/test/test_size_gate.py` → `7 passed`。

---

## C. L2 建图端：静态世界门

实现位于纯逻辑模块 `cone_mapping/cone_mapper.py`（无 ROS 依赖），节点只做接线。

### C1. Tentative → confirmed 生命周期 ✅（单元测试验证）

- **新位置必先 tentative**：不可发布（`publishable` 过滤，`/mapping/cone_map` 只含 established）、独立存活 `tentative_max_age_sec = 5.0 s`，超时由 `evict_stale` 驱逐，不留残余（测试 `test_tentative_evicted_quickly`）。
- **转正条件**：`confirm_min_obs = 4` 次「紧观测」（落入条目当前位置 `confirm_radius = 0.15 m` 内）且**同色**（异色/UNKNOWN 观测不计入，`test_C1_confirm_requires_matching_colour`），时间跨度 ≥ `confirm_span_sec = 1.0 s`（从 first_seen 到 last_seen）。注意转正判定的触发时机在第 K 次紧观测到达时——只在短暂停留的源上攒够次数、随后观测变松的源（走动人）永远卡在 K 以下（`test_walking_source_never_promotes`）。
- **取值理由（bench/推车）**：bench 检测频率 ~5–10 Hz（推理 0.1–0.3 s 自节拍）→ 4 次 × 1 s ≈ 7–10 帧即时延迟，静态摆拍 1 s 即入图，可接受；推车速度 ≤ 5 m/s 时帧间位移 ≤ 0.5–0.7 m，0.15 m 紧半径对 5 m 内锥桶的 ZED 深度噪声（≈2–4% × z）仍偏紧，远桶转正会稍慢（多等 1–2 s），属保守方向（慢入图好于错入图）；真桶被持续看到时该延迟一次性发生，不影响后续。

### C2. Innovation 门（防拖拽）✅（单元测试验证）

- **门限**：`d_accept(n) = min(2.5 × σ(n), 0.4 m)`，其中 `σ(n) = sqrt(max(0.01, 0.25/n))` —— 与发布协方差 `covariance_for` 同源的**建图端内部 σ**（观测均值的不确定度，随观测数收缩，0.1 m 地板）。
- **σ 来源交代**：
  - **未采用**消息自带协方差（检测端 `_range_std_for(z) = max(0.05, 0.01z)`，轴对角）。按清单要求换算 2.5σ 门限供审：

    | 距离 | σ | 2.5σ |
    |---|---|---|
    | 3 m | 0.05 m（下限生效） | **0.125 m** |
    | 5 m | 0.05 m | **0.125 m** |
    | 8 m | 0.08 m | **0.20 m** |

    3–5 m 处门限仅 12.5 cm 且无标定依据——按清单标准「视为未过」，故弃用。
  - **采用**内部 σ + 0.1 m 地板：门限按观测数收缩，距离无关（mapper 无相机距离信息），无标定数据，**标注：待标定**。门限米数表（按 obs_count）：

    | obs_count | σ | 门限 d_accept |
    |---|---|---|
    | 4–14（年轻） | 0.25–0.13 m | **0.40 m（cap 生效）** |
    | 16 | 0.125 m | 0.3125 m |
    | ≥25（成熟） | 0.10 m（地板） | **0.25 m** |

  - **标定计划（待执行）**：bench 静态推车各跑一遍，记录每条已确立条目的「接受观测偏移量 |obs−entry| – obs_count – 检测端 z」；拟给 `ConeDetectionStamped` 补充检测距离字段（或在检测端旁路记录），拟合 σ(range)，替换 0.1 m 地板与 cap。标定前 0.25–0.40 m 的门限带是「防拖拽 vs 正常深度噪声」的折中先验。
- **拖拽语义**：门限内观测正常合并（加权平均）；门限外观测**不并入**（条目位置/obs_count 不动），改为在观测处生成 tentative——真实持久源会在那里转正并由 C3 退役被饥饿的旧条目，瞬态源随 5 s tentative 驱逐消失。
- **单元测试**（`test_cone_mapper.py`）：
  - `test_C2_established_not_dragged_by_bad_stream`：n=30 条目 + 10 个 0.4 m 偏移坏观测 → 位置逐字节不动、obs_count 不变、tentative 出现。
  - `test_C2_young_established_not_dragged_by_bad_stream`：**年轻条目**（n=5，σ 门限本应 0.559 m > 合并半径）+ 10 个 0.45 m 坏观测 → 不动（cap 0.4 生效）。
  - `test_C2_young_established_still_merges_normal_observations`：年轻条目 0.2 m 正常观测照常合并。
  - `test_C2_cap_bounds_young_entries_sigma_floors_mature_ones`：门限曲线 cap/σ收缩/地板三段数值断言。
  - `test_established_entry_accepts_within_gate`：门限内正常合并。

### C3. 冲突退役（锥被真实移动）✅（单元测试验证）

- **触发条件（周期执行，publish 时 10 Hz 调 `retire_superseded`）**：established 条目①超过 `stale_conflict_sec = 5.0 s` 没有接受任何观测（「饥饿」——真锥在视野内会持续收到观测）**且** ② `conflict_radius = 1.5 m` 内存在**同色、非 UNKNOWN、且仍在被观测**（≤5 s 内有接受观测）的 established 条目 → 旧条目移除，打 WARN 日志。
- **为何做成周期性而非转正瞬间**：挪远 >0.5 m 时旧条目不会再收到任何关联观测（也就没有"持续拒收"计数可依据），转正瞬间旧条目往往还不满足 5 s 饥饿；周期检查统一覆盖近移/远移，且要求"孪生"必须走完完整 C1 转正，防瞬态幽灵退役真条目。两个都饥饿的条目互不退役（`test_two_starved_entries_do_not_retire_each_other`）。
- **时序（挪锥 ~1 m 场景，测试 `test_C3_cone_moved_1m_becomes_only_published_entry`）**：新位置首个观测 t≈0.1 s → tentative → **t≈1.2 s 转正**（对规划器可见）→ 旧条目 t≈5.0 s 起满足饥饿 → **t≈5–6 s 退役**。即 ~6 s 后新位置成为唯一发布条目，旧位置无残留。
- **保护条件**：异色不退（`test_keeps_stale_different_colour_established`）、新鲜不退（`test_keeps_fresh_same_colour_established`）、超出 1.5 m 不退（`test_keeps_stale_established_beyond_conflict_radius`）。
- **已知边界（诚实交代）**：若挪移 >1.5 m，旧条目无同色孪生可依，只能等 `max_age_sec`（bench 120 s）驱逐——`conflict_radius` 再大会威胁真实相邻锥（FSAC 同色最小间距 ~1 m 量级），故取 1.5 m。

---

## D. 参数纪律

- **max_age_sec**：代码默认 `120.0` **未动**（本任务从未改过默认值；原 launch 的 600 是 bench 覆盖值）。bench launch 由 600 → 120。
  - 是否收紧代码默认值的**单独论证**：不建议。正式跑圈时车尾方向锥桶长时间不回视（trackdrive 单向行驶），默认 120 s 足够安全；再短（如 60 s）会在长直道/回旋段把身后真桶驱逐，导致规划器丢图。bench 收紧到 120 与默认一致，无额外风险。
- **「拒收 <1 m 观测」**：**未实现**。理由：上车后近距离过桶（<1 m）真实存在，硬拒会丢近桶；L1 的深度–尺寸门已在源头拒绝"深度采到手/人"的观测（手在 0.5–1 m 时 z 会被采成手距，比值矛盾被丢），不需要按距离一刀切。若 bench 实测仍有近距离幽灵，作为独立参数（bench-only）追加。
- **min_observations**：代码默认 `3` **未动**；bench launch 设 `5`。理由：C1 转正已要求 4 次紧观测跨 1 s，established 条目最少 4 obs；5 = 转正后再多 1 帧证据，延迟代价 ~0.1–0.2 s。**未用**它替代 B/C 机制（仅是发布侧的最后一道阈值）。

---

## E. 场景回归（核心证据）——⏳ 全部未测

**未测原因**：本会话为无 ZED 硬件、无 Gazebo 的代码工作区，无法产生实测数据。
以下为执行手册（改前基线数据亦需按此采集，Muse/使用者在 bench 上执行）：

### E-① 遮挡复现（改前/改后）

```bash
colcon build --packages-select cone_mapping yolo_cone_detection
ros2 launch yolo_cone_detection zed_real_pipeline.launch.py
# RViz 固定视角，观察 /mapping/cone_map_viz；另开终端：
ros2 topic echo /mapping/cone_map --once   # 记录真桶条目 (x, y)
# 手（或木板）遮挡同一锥桶 ~1 s 后移开，观察 60 s
```
- **改前预期**：移开时出现新锥桶条目（幽灵）；真桶条目可能被拖离原位（对应 2026-11-01 实测现象，可留录屏作为基线证据）。
- **改后通过标准**：地图幽灵数 = 0；真桶条目位置偏移 ≤ 0.2 m；检测端日志 `size_gate_dropped` 在遮挡期间增长。
- **结果**：未测。

### E-② 人员横穿

- 同场景，人从相机前 ~1–3 m 横穿（有橙色衣物更佳，模拟裁判马甲）。
- **改后通过标准**：`/mapping/cone_map` 不出现跟随人的条目；允许出现灰色 tentative 小球（`cone_map_tentative` namespace）跟随人，人离开后 ≤5 s 消失；橙色衣物被检成 orange cone 时检测端日志 `size_gate_dropped` 增长。
- **结果**：未测。

### E-③ 无遮挡基线

- 同场景无干扰跑 ≥3 min，对比改前/改后的真桶检出数与入图数（echo `/mapping/cone_map` 统计 + 检测端日志 `cones=` 均值）。
- **通过标准**：差异 ≤5%；>5% 须归因（候选原因：L1 对贴边近距锥的豁免是否充分、min_observations 3→5 的入图延迟）。
- **结果**：未测。

### E-④ EUFS 仿真冒烟

```bash
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py   # 或现有仿真入口
```
- **通过标准**：检测与建图正常出图，`/mapping/cone_map` 锥桶数与仿真赛道一致，无异常刷屏（重点看 size_gate/gate/retirement 相关 WARN 频率）。
- **结果**：未测。

---

## 单元测试汇总（A-④ 通过记录）

```
pytest src/cone_mapping/test/test_cone_mapper.py      -> 33 passed
pytest src/yolo_cone_detection/test/test_size_gate.py ->  7 passed
py_compile 全部改动文件 -> OK；yolo_detector.yaml yaml.safe_load -> OK
```

测试名与清单条目对应关系见 B-⑤ 表与 C 节各 `test_C*` 前缀用例。

## 残余风险与待办

1. **待标定**：L1 容差带（0.55/2.0）、C2 门限带（0.25–0.40 m）均为先验，标定计划见 B-②/C2。
2. L1 共享容差带对蓝/黄类的上界偏松（0.61 m 视高内不拒），见 B-① slack 说明。
3. 挪锥 >1.5 m 时旧条目等 max_age 驱逐（bench 120 s），见 C3 边界。
4. 几何上完全自洽的稳态遮挡、蹲姿人形仍属语义盲区（清单外，靠 L2 拖延 + tentative 可视化暴露）。
5. E 节四项场景回归待 bench 执行后回填。
