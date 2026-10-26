"""Dashboard page: system control, mission control, vehicle state, ROS2 status."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QWidget, QGridLayout
)

from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui.dashboard")

# Status label helper ------------------------------------------------------
def _status_label(text: str, ok: bool | None) -> QLabel:
    label = QLabel(text)
    label.setAlignment(Qt.AlignCenter)
    if ok is True:
        label.setStyleSheet("color: green; font-weight: bold;")
    elif ok is False:
        label.setStyleSheet("color: red; font-weight: bold;")
    else:
        label.setStyleSheet("color: gray;")
    return label


class DashboardPage(QWidget):
    """Top-level overview page.

    Left:   system + mission control buttons
    Middle: vehicle state values
    Right:  ROS2 / subsystem status indicators
    """

    # Control signals emitted to main_window.
    start_sim_requested = Signal()
    stop_sim_requested = Signal()
    restart_sim_requested = Signal()
    emergency_stop_requested = Signal()
    start_mission_requested = Signal(int)  # mission id
    pause_mission_requested = Signal()
    reset_mission_requested = Signal()

    # Launch mode identifiers (stored as combo-box item data).
    MODE_MINIMAL = "minimal"
    MODE_YOLO_AUTONOMOUS = "yolo_autonomous"

    # Available tracks: (english_id, chinese_name)
    TRACKS = [
        ("small_oval", "小椭圆赛道"),
        ("small_track", "小赛道"),
        ("small_track_2", "小赛道2"),
        ("acceleration", "加速赛赛道"),
        ("skidpad", "八字绕桩赛道"),
        ("peanut", "花生形赛道"),
        ("B_shape_02_03_2023", "B形赛道"),
        ("Hairpin_02_03_2023", "发卡弯赛道"),
        ("Jellybean_02_03_2023", "糖豆形赛道"),
        ("BM_long_straight", "长直道赛道"),
        ("BM_text_bubble", "文字气泡赛道"),
        ("FSDS_Training", "FSDS训练赛道"),
        ("QR_Nov_2022", "QR赛道"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()
        # Apply initial mode-dependent widget state.
        self._on_mode_changed(self.cmb_launch_mode.currentIndex())

    def _build_ui(self):
        root = QHBoxLayout(self)

        # --- Left column: controls ----------------------------------------
        left = QVBoxLayout()

        sys_group = QGroupBox("System Control")
        sys_layout = QVBoxLayout(sys_group)

        # Launch mode selector.
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self.cmb_launch_mode = QComboBox()
        self.cmb_launch_mode.addItem("Minimal Control Loop", self.MODE_MINIMAL)
        self.cmb_launch_mode.addItem(
            "YOLO Autonomous (camera)", self.MODE_YOLO_AUTONOMOUS
        )
        self.cmb_launch_mode.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self.cmb_launch_mode)
        sys_layout.addLayout(mode_layout)

        # Track selector (Chinese names).
        track_layout = QHBoxLayout()
        track_layout.addWidget(QLabel("赛道:"))
        self.cmb_track = QComboBox()
        for eng, chn in self.TRACKS:
            self.cmb_track.addItem(chn, eng)
        track_layout.addWidget(self.cmb_track)
        sys_layout.addLayout(track_layout)

        # TCone-YOLO path (only relevant for YOLO mode).
        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("YOLO path:"))
        self.txt_yolo_path = QLineEdit()
        self.txt_yolo_path.setPlaceholderText("/path/to/TCone-YOLO-main")
        self.txt_yolo_path.setText("~/src10.22/src/TCone-YOLO-main")
        path_layout.addWidget(self.txt_yolo_path)
        sys_layout.addLayout(path_layout)

        self.btn_start_sim = QPushButton("Start Simulator")
        self.btn_stop_sim = QPushButton("Stop Simulator")
        self.btn_restart_sim = QPushButton("Restart Simulator")
        self.btn_start_sim.clicked.connect(self.start_sim_requested.emit)
        self.btn_stop_sim.clicked.connect(self.stop_sim_requested.emit)
        self.btn_restart_sim.clicked.connect(self.restart_sim_requested.emit)
        sys_layout.addWidget(self.btn_start_sim)
        sys_layout.addWidget(self.btn_stop_sim)
        sys_layout.addWidget(self.btn_restart_sim)

        self.lbl_ros2_status = _status_label("ROS2: UNKNOWN", None)
        self.lbl_gui_status = _status_label("GUI: RUNNING", True)
        sys_layout.addWidget(self.lbl_ros2_status)
        sys_layout.addWidget(self.lbl_gui_status)
        left.addWidget(sys_group)

        mission_group = QGroupBox("Mission Control")
        mission_layout = QVBoxLayout(mission_group)
        self.btn_manual = QPushButton("Manual")
        self.btn_autonomous = QPushButton("Autonomous")
        self.btn_start_mission = QPushButton("Start Mission")
        self.btn_pause_mission = QPushButton("Pause Mission")
        self.btn_reset_mission = QPushButton("Reset Mission")
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: #cc0000; color: white; font-weight: bold; padding: 12px;"
        )
        self.btn_manual.clicked.connect(lambda: self.start_mission_requested.emit(1))
        self.btn_autonomous.clicked.connect(lambda: self.start_mission_requested.emit(4))
        self.btn_start_mission.clicked.connect(lambda: self.start_mission_requested.emit(4))
        self.btn_pause_mission.clicked.connect(self.pause_mission_requested.emit)
        self.btn_reset_mission.clicked.connect(self.reset_mission_requested.emit)
        self.btn_emergency.clicked.connect(self.emergency_stop_requested.emit)
        mission_layout.addWidget(self.btn_manual)
        mission_layout.addWidget(self.btn_autonomous)
        mission_layout.addWidget(self.btn_start_mission)
        mission_layout.addWidget(self.btn_pause_mission)
        mission_layout.addWidget(self.btn_reset_mission)
        mission_layout.addWidget(self.btn_emergency)
        left.addWidget(mission_group)
        left.addStretch()

        root.addLayout(left)

        # --- Middle column: vehicle state ---------------------------------
        mid = QVBoxLayout()
        veh_group = QGroupBox("Vehicle State")
        veh_grid = QGridLayout(veh_group)
        self.lbl_speed = QLabel("Speed: NOT AVAILABLE")
        self.lbl_steering = QLabel("Steering: NOT AVAILABLE")
        self.lbl_throttle = QLabel("Throttle: NOT AVAILABLE")
        self.lbl_brake = QLabel("Brake: NOT AVAILABLE")
        self.lbl_x = QLabel("X: NOT AVAILABLE")
        self.lbl_y = QLabel("Y: NOT AVAILABLE")
        self.lbl_yaw = QLabel("Yaw: NOT AVAILABLE")
        for i, w in enumerate(
            [self.lbl_speed, self.lbl_steering, self.lbl_throttle, self.lbl_brake,
             self.lbl_x, self.lbl_y, self.lbl_yaw]
        ):
            veh_grid.addWidget(w, i, 0)
        mid.addWidget(veh_group)
        mid.addStretch()
        root.addLayout(mid)

        # --- Right column: ROS2 / subsystem status -----------------------
        right = QVBoxLayout()
        ros_group = QGroupBox("ROS2 Status")
        ros_grid = QGridLayout(ros_group)
        self.lbl_nodes = QLabel("Nodes: -")
        self.lbl_topics = QLabel("Topics: -")
        self.lbl_services = QLabel("Services: -")
        self.lbl_actions = QLabel("Actions: -")
        self.lbl_tf = _status_label("TF: NOT TESTED", None)
        self.lbl_controller = _status_label("Controller: NOT TESTED", None)
        self.lbl_perception = _status_label("Perception: NOT TESTED", None)
        self.lbl_navigation = _status_label("Navigation: NOT TESTED", None)
        widgets = [
            self.lbl_nodes, self.lbl_topics, self.lbl_services, self.lbl_actions,
            self.lbl_tf, self.lbl_controller, self.lbl_perception, self.lbl_navigation,
        ]
        for i, w in enumerate(widgets):
            ros_grid.addWidget(w, i, 0)
        right.addWidget(ros_group)
        right.addStretch()
        root.addLayout(right)

    # --- Public update slots called from main_window ----------------------
    def set_ros2_connected(self, connected: bool):
        self.lbl_ros2_status.setText(f"ROS2: {'CONNECTED' if connected else 'DISCONNECTED'}")
        self.lbl_ros2_status.setStyleSheet(
            "color: green; font-weight: bold;" if connected else "color: red; font-weight: bold;"
        )

    def update_graph(self, graph: dict):
        self.lbl_nodes.setText(f"Nodes: {graph.get('node_count', 0)}")
        self.lbl_topics.setText(f"Topics: {graph.get('topic_count', 0)}")
        self.lbl_services.setText(f"Services: {graph.get('service_count', 0)}")
        self.lbl_actions.setText(f"Actions: {graph.get('action_count', 0)}")

    def update_vehicle(self, state: dict):
        v = state.get("velocity")
        s = state.get("steering")
        self.lbl_speed.setText(f"Speed: {v:.3f} m/s" if v is not None else "Speed: NOT AVAILABLE")
        self.lbl_steering.setText(f"Steering: {s:.3f} deg" if s is not None else "Steering: NOT AVAILABLE")
        x = state.get("x")
        y = state.get("y")
        yaw = state.get("yaw")
        self.lbl_x.setText(f"X: {x:.3f}" if x is not None else "X: NOT AVAILABLE")
        self.lbl_y.setText(f"Y: {y:.3f}" if y is not None else "Y: NOT AVAILABLE")
        self.lbl_yaw.setText(f"Yaw: {yaw:.3f} rad" if yaw is not None else "Yaw: NOT AVAILABLE")

    def update_control(self, control: dict):
        speed = control.get("speed")
        accel = control.get("acceleration")
        self.lbl_throttle.setText(f"Throttle: {speed:.3f}" if speed is not None else "Throttle: NOT AVAILABLE")
        # No dedicated brake topic; show acceleration (negative = braking).
        self.lbl_brake.setText(
            f"Brake(accel): {accel:.3f}" if accel is not None else "Brake: NOT AVAILABLE"
        )

    def update_subsystem_status(self, system: dict):
        self.lbl_controller.setText(
            f"Controller: {'PASS' if system.get('good_to_go') else 'NOT TESTED'}"
        )
        self.lbl_controller.setStyleSheet(
            "color: green;" if system.get("good_to_go") else "color: gray;"
        )
        self.lbl_perception.setText(
            f"Perception: {'PASS' if system.get('lidar_operational') else 'NOT TESTED'}"
        )
        self.lbl_perception.setStyleSheet(
            "color: green;" if system.get("lidar_operational") else "color: gray;"
        )
        self.lbl_navigation.setText(
            f"Navigation: {'PASS' if system.get('planning') else 'NOT TESTED'}"
        )
        self.lbl_navigation.setStyleSheet(
            "color: green;" if system.get("planning") else "color: gray;"
        )

    # --- Launch mode accessors -------------------------------------------
    @property
    def launch_mode(self) -> str:
        """Return the currently selected launch mode identifier."""
        return self.cmb_launch_mode.currentData()

    @property
    def tcone_yolo_path(self) -> str:
        """Return the TCone-YOLO path with ~ expanded."""
        import os
        return os.path.expanduser(self.txt_yolo_path.text().strip())

    @property
    def track(self) -> str:
        """Return the currently selected track's English identifier."""
        return self.cmb_track.currentData()

    def _on_mode_changed(self, _index: int):
        """Enable/disable the YOLO path field based on the selected mode."""
        is_yolo = self.launch_mode == self.MODE_YOLO_AUTONOMOUS
        self.txt_yolo_path.setEnabled(is_yolo)
        if not is_yolo:
            self.txt_yolo_path.setStyleSheet(
                "color: gray; background-color: #f0f0f0;"
            )
        else:
            self.txt_yolo_path.setStyleSheet("")