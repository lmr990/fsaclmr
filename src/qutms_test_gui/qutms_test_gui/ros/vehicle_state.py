"""Vehicle state subscriber.

Subscribes to the real vehicle-state and control-command topics confirmed
from source code, and forwards the latest values to the GUI through Qt
signals. All subscriptions are created on the RosManager node so callbacks
run inside the rclpy spin thread.

Topics subscribed (all VERIFIED from source):
  /vehicle/velocity            std_msgs/Float32            (m/s)
  /vehicle/steering_angle      std_msgs/Float32            (degrees)
  /odometry/sbg_ekf            nav_msgs/Odometry           (pose + twist)
  /control/driving_command     ackermann_msgs/AckermannDriveStamped
  /system/av_state             driverless_msgs/AVStateStamped
  /system/ros_state            driverless_msgs/ROSStateStamped
"""

import math
import threading
import time

from PySide6.QtCore import QObject, Signal

from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui.vehicle_state")

# Topics that the GUI vehicle page cares about, with confirmed message types.
VEHICLE_TOPICS = [
    ("/vehicle/velocity", "std_msgs.msg.Float32"),
    ("/vehicle/steering_angle", "std_msgs.msg.Float32"),
    ("/odometry/sbg_ekf", "nav_msgs.msg.Odometry"),
    ("/control/driving_command", "ackermann_msgs.msg.AckermannDriveStamped"),
    ("/system/av_state", "driverless_msgs.msg.AVStateStamped"),
    ("/system/ros_state", "driverless_msgs.msg.ROSStateStamped"),
]


class VehicleState(QObject):
    """Subscribes to vehicle state topics and emits GUI-friendly snapshots."""

    vehicle_updated = Signal(dict)
    control_updated = Signal(dict)
    system_state_updated = Signal(dict)

    STALE_THRESHOLD_SEC = 1.0

    def __init__(self, ros_manager):
        super().__init__()
        self._ros = ros_manager
        self._subs = {}
        self._state = {
            "velocity": None,
            "steering": None,
            "x": None,
            "y": None,
            "yaw": None,
            "last_vehicle_update": 0.0,
        }
        self._control = {
            "steering_angle": None,
            "speed": None,
            "acceleration": None,
            "last_control_update": 0.0,
        }
        self._system = {
            "av_mode": None,
            "av_state": None,
            "mission": None,
            "good_to_go": None,
            "lidar_operational": None,
            "planning": None,
            "sbg_operational": None,
            "finished": None,
            "lap_count": None,
        }
        self._lock = threading.Lock()

    def start(self):
        if not self._ros.ros2_ok or self._ros.node is None:
            logger.warn("ROS2 not available; vehicle state subscriber not started")
            return

        for topic, type_path in VEHICLE_TOPICS:
            try:
                msg_type = self._import_type(type_path)
                cb = self._make_callback(topic)
                sub = self._ros.node.create_subscription(msg_type, topic, cb, 10)
                self._subs[topic] = sub
                logger.info("Subscribed to %s", topic)
            except Exception as exc:
                logger.error("Failed to subscribe to %s: %s", topic, exc)

    def stop(self):
        for sub in self._subs.values():
            try:
                self._ros.node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs.clear()

    def _make_callback(self, topic):
        if topic == "/vehicle/velocity":
            return self._on_velocity
        if topic == "/vehicle/steering_angle":
            return self._on_steering
        if topic == "/odometry/sbg_ekf":
            return self._on_odometry
        if topic == "/control/driving_command":
            return self._on_driving_command
        if topic == "/system/av_state":
            return self._on_av_state
        if topic == "/system/ros_state":
            return self._on_ros_state
        return lambda msg: None

    def _on_velocity(self, msg):
        with self._lock:
            self._state["velocity"] = float(msg.data)
            self._state["last_vehicle_update"] = time.time()
            self.vehicle_updated.emit(dict(self._state))

    def _on_steering(self, msg):
        with self._lock:
            self._state["steering"] = float(msg.data)
            self._state["last_vehicle_update"] = time.time()
            self.vehicle_updated.emit(dict(self._state))

    def _on_odometry(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._state["x"] = msg.pose.pose.position.x
            self._state["y"] = msg.pose.pose.position.y
            self._state["yaw"] = yaw
            self._state["last_vehicle_update"] = time.time()
            self.vehicle_updated.emit(dict(self._state))

    def _on_driving_command(self, msg):
        with self._lock:
            self._control["steering_angle"] = float(msg.drive.steering_angle)
            self._control["speed"] = float(msg.drive.speed)
            self._control["acceleration"] = float(msg.drive.acceleration)
            self._control["last_control_update"] = time.time()
            self.control_updated.emit(dict(self._control))

    def _on_av_state(self, msg):
        with self._lock:
            self._system["av_mode"] = int(msg.mode)
            self._system["av_state"] = int(msg.state)
            self._system["mission"] = int(msg.mission)
            self.system_state_updated.emit(dict(self._system))

    def _on_ros_state(self, msg):
        with self._lock:
            self._system["good_to_go"] = bool(msg.good_to_go)
            self._system["lidar_operational"] = bool(msg.lidar_operational)
            self._system["planning"] = bool(msg.planning)
            self._system["sbg_operational"] = bool(msg.sbg_operational)
            self._system["finished"] = bool(msg.finished)
            self._system["lap_count"] = int(msg.lap_count)
            self.system_state_updated.emit(dict(self._system))

    @staticmethod
    def _import_type(dotted_path: str):
        parts = dotted_path.rsplit(".", 1)
        module_path, class_name = parts
        module = __import__(module_path, fromlist=[class_name])
        return getattr(module, class_name)
