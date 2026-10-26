# webcam_path_demo — 真实摄像头锥桶检测 + 路径叠加显示

用笔记本摄像头识别真实锥桶，经单目测距后送入项目原有的 cone_mapper + ft_planner
规划中线，并把路径实时画回摄像头画面。无需仿真器。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
source ~/qutms_ws/install/setup.bash

# 推荐：真实 ZED 相机 + 双目深度 + 检测/路径叠加（ZED-M 已验证）
ros2 launch webcam_path_demo zed_path_demo.launch.py

# 笔记本摄像头 + 单目测距
ros2 launch webcam_path_demo webcam_path_demo.launch.py

# 常用变体
ros2 launch webcam_path_demo webcam_path_demo.launch.py display:=false \
    save_debug_png:=/tmp/overlay.png          # 不弹窗，把叠加图存文件
ros2 launch webcam_path_demo webcam_path_demo.launch.py distance_method:=ground \
    camera_height:=0.35 camera_pitch_deg:=15  # 地平面法测距（更准，需量相机高度/俯角）
ros2 launch webcam_path_demo webcam_path_demo.launch.py source:=file \
    image_files:="/path/to/*.png" intrinsics_file:=~/webcam_intrinsics.yaml
```

按 `q` 退出显示窗口。画面内容：检测框（按锥桶颜色着色+置信度）、绿点=测距锚点、
绿线=规划中线、黄点=路径采样点，左上角显示锥桶数/单帧检测耗时/是否有路径。

## 提高精度的两步（可选但推荐）

1. **相机标定**（打印一张 9x6 棋盘格，25mm 方格）：

   ```bash
   ros2 run webcam_path_demo webcam_calibrate --output ~/webcam_intrinsics.yaml
   # 对着棋盘格从不同角度凑够 15 帧，自动保存
   ros2 launch webcam_path_demo webcam_path_demo.launch.py intrinsics_file:=~/webcam_intrinsics.yaml
   ```

2. **地平面测距**：量一下镜头离地高度和下俯角度，`distance_method:=ground`
   （横向和远距离精度都比默认的尺寸法好）。

## 摆锥桶要求

- 蓝/黄锥桶分列两侧（行驶方向蓝左黄右，FSAC 规则），同色至少 2-3 个
- 最近一对锥桶距摄像头 **6 米以内**（planner 限制 `max_dist_to_first: 6.0`）
- 距离由 `cone_real_height`（默认 0.325m，FSAC 小桶）反算 —— 用小玩具桶演示时
  把它改成实际高度，否则距离会按比例偏差

## 三种图像源（source 参数）

| source | 输入 | 测距 | 场景 |
|--------|------|------|------|
| `topic`（推荐） | ZED wrapper v5 话题 | **真实双目深度**（无效区退回单目） | ZED-M/2i 实机 |
| `camera` | cv2 笔记本摄像头 | 单目（尺寸法/地平面法） | 无 ZED 时 |
| `file` | 图片 glob 循环回放 | 同上 | 离线调试 |

叠加图状态栏：`cones=检测数 detect=推理ms depth=zed/est path=yes/no`。

## 架构

```
webcam_path_node ──slam/cone_detection──> cone_mapper_node ──/mapping/cone_map──> ft_planner_node
       │                                                                                   │
       └──────────────── /planning/midline_path <──────────────────────────────────────────┘
       （叠加绘制回图像）
静态 TF: track -> base_link（车=原点）   mock /vehicle/velocity=0（速度播种）
```

复用组件：`yolo_cone_detection.yolo_inference`（TCone-YOLO GPU 推理）、
`cone_mapping`、`planners`（均未改动）。坐标约定与仿真节点一致（REP-103：+y 左）。

## 自测

```bash
# 无摄像头无GPU的规划链自检：合成30m蓝黄走廊，验证 planner 输出与理论中线偏差
python3 ~/qutms_ws/diagnostics/synthetic_corridor_test.py
```

## 已知限制

- 单目测距（尤其尺寸法）径向误差 ±10-20%，路径形状会随距离误差变形 —— 演示用足够，
  不能用于控制实车
- 强逆光/锥桶颜色与背景撞色会降低检出率，可调 `conf_thres`
- 发车线式摆桶（一排橙桶+远处零星蓝黄）不构成走廊，planner 会拒绝或规划出怪路径，
  属预期行为
