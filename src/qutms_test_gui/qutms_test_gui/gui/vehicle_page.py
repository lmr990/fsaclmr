"""Vehicle page: detailed vehicle state and control command display."""

import time

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QVBoxLayout, QWidget
)


class VehiclePage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        state_group = QGroupBox("Vehicle State")
        state_grid = QGridLayout(state_group)
        self.lbl_speed = QLabel("Speed: NOT AVAILABLE")
        self.lbl_steering = QLabel("Steering: NOT AVAILABLE")
        self.lbl_x = QLabel("X: NOT AVAILABLE")
        self.lbl_y = QLabel("Y: NOT AVAILABLE")
        self.lbl_yaw = QLabel("Yaw: NOT AVAILABLE")
        self.lbl_state_update = QLabel("Last update: -")
        for i, w in enumerate([self.lbl_speed, self.lbl_steering, self.lbl_x,
                                self.lbl_y, self.lbl_yaw, self.lbl_state_update]):
            state_grid.addWidget(w, i, 0)
        layout.addWidget(state_group)

        ctrl_group = QGroupBox("Control Command (/control/driving_command)")
        ctrl_grid = QGridLayout(ctrl_group)
        self.lbl_cmd_steering = QLabel("Steering: NOT AVAILABLE")
        self.lbl_cmd_speed = QLabel("Speed: NOT AVAILABLE")
        self.lbl_cmd_accel = QLabel("Acceleration: NOT AVAILABLE")
        self.lbl_cmd_update = QLabel("Last update: -")
        for i, w in enumerate([self.lbl_cmd_steering, self.lbl_cmd_speed,
                                self.lbl_cmd_accel, self.lbl_cmd_update]):
            ctrl_grid.addWidget(w, i, 0)
        layout.addWidget(ctrl_group)
        layout.addStretch()

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
        last = state.get("last_vehicle_update", 0)
        if last > 0:
            age = time.time() - last
            color = "red" if age > 1.0 else "green"
            self.lbl_state_update.setText(f"Last update: {age:.2f} s ago")
            self.lbl_state_update.setStyleSheet(f"color: {color};")

    def update_control(self, control: dict):
        sa = control.get("steering_angle")
        sp = control.get("speed")
        ac = control.get("acceleration")
        self.lbl_cmd_steering.setText(f"Steering: {sa:.3f} deg" if sa is not None else "Steering: NOT AVAILABLE")
        self.lbl_cmd_speed.setText(f"Speed: {sp:.3f} m/s" if sp is not None else "Speed: NOT AVAILABLE")
        self.lbl_cmd_accel.setText(f"Acceleration: {ac:.3f}" if ac is not None else "Acceleration: NOT AVAILABLE")
        last = control.get("last_control_update", 0)
        if last > 0:
            age = time.time() - last
            color = "red" if age > 1.0 else "green"
            self.lbl_cmd_update.setText(f"Last update: {age:.2f} s ago")
            self.lbl_cmd_update.setStyleSheet(f"color: {color};")
