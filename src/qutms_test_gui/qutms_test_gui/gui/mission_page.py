"""Mission page: mission control buttons, status, and emergency stop.

The page only emits signals; the actual ROS2 publishing / service calls
are performed by main_window using the RosManager node. Interfaces that
are not confirmed are shown as NOT CONFIGURED.
"""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
)


class MissionPage(QWidget):
    start_mission_requested = Signal(int)
    pause_mission_requested = Signal()
    reset_mission_requested = Signal()
    emergency_stop_requested = Signal()

    # Mission ids match driverless_msgs/State.msg enum.
    MISSIONS = [
        (0, "None"),
        (1, "Manual Driving"),
        (2, "Inspection"),
        (3, "EBS Test"),
        (4, "Trackdrive"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        ctrl_group = QGroupBox("Mission Control")
        ctrl_layout = QVBoxLayout(ctrl_group)

        # Mission selection buttons.
        btn_row = QHBoxLayout()
        for mid, name in self.MISSIONS:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked=False, m=mid: self.start_mission_requested.emit(m))
            btn_row.addWidget(btn)
        ctrl_layout.addLayout(btn_row)

        self.btn_start = QPushButton("Start Mission")
        self.btn_pause = QPushButton("Pause Mission")
        self.btn_reset = QPushButton("Reset Mission")
        self.btn_start.clicked.connect(lambda: self.start_mission_requested.emit(4))
        self.btn_pause.clicked.connect(self.pause_mission_requested.emit)
        self.btn_reset.clicked.connect(self.reset_mission_requested.emit)
        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_pause)
        ctrl_layout.addWidget(self.btn_reset)

        self.lbl_pause_status = QLabel("Pause interface: NOT CONFIGURED")
        self.lbl_pause_status.setStyleSheet("color: orange;")
        ctrl_layout.addWidget(self.lbl_pause_status)
        layout.addWidget(ctrl_group)

        ebs_group = QGroupBox("Emergency")
        ebs_layout = QVBoxLayout(ebs_group)
        self.btn_emergency = QPushButton("EMERGENCY STOP")
        self.btn_emergency.setStyleSheet(
            "background-color: #cc0000; color: white; font-weight: bold; padding: 16px; font-size: 16px;"
        )
        self.btn_emergency.clicked.connect(self.emergency_stop_requested.emit)
        ebs_layout.addWidget(self.btn_emergency)
        self.lbl_ebs_interface = QLabel(
            "E-stop interface: /system/shutdown (Shutdown.emergency_shutdown) "
            "+ zero-speed /control/driving_command"
        )
        ebs_layout.addWidget(self.lbl_ebs_interface)
        layout.addWidget(ebs_group)

        status_group = QGroupBox("System State")
        status_layout = QVBoxLayout(status_group)
        self.lbl_mode = QLabel("AV Mode: -")
        self.lbl_state = QLabel("AV State: -")
        self.lbl_mission = QLabel("Mission: -")
        self.lbl_g2g = QLabel("Good to go: -")
        self.lbl_laps = QLabel("Laps: -")
        for w in [self.lbl_mode, self.lbl_state, self.lbl_mission,
                  self.lbl_g2g, self.lbl_laps]:
            status_layout.addWidget(w)
        layout.addWidget(status_group)
        layout.addStretch()

    def update_system_state(self, system: dict):
        mode = system.get("av_mode")
        state = system.get("av_state")
        mission = system.get("mission")
        self.lbl_mode.setText(f"AV Mode: {mode if mode is not None else '-'}")
        self.lbl_state.setText(f"AV State: {state if state is not None else '-'}")
        self.lbl_mission.setText(f"Mission: {mission if mission is not None else '-'}")
        self.lbl_g2g.setText(f"Good to go: {system.get('good_to_go')}")
        self.lbl_laps.setText(f"Laps: {system.get('lap_count')}")
