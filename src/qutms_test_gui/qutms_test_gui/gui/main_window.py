"""Main window: assembles all pages and wires ROS2 / process / GUI signals."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QMainWindow, QTabWidget, QVBoxLayout, QWidget, QStatusBar
)

from qutms_test_gui.gui.dashboard import DashboardPage
from qutms_test_gui.gui.nodes_page import NodesPage
from qutms_test_gui.gui.topics_page import TopicsPage
from qutms_test_gui.gui.vehicle_page import VehiclePage
from qutms_test_gui.gui.mission_page import MissionPage
from qutms_test_gui.gui.logs_page import LogsPage

from qutms_test_gui.ros.ros_manager import RosManager
from qutms_test_gui.ros.topic_monitor import TopicMonitor
from qutms_test_gui.ros.vehicle_state import VehicleState
from qutms_test_gui.process.process_manager import ProcessManager

from qutms_test_gui.logger import setup_logger, get_log_signal

logger = setup_logger("qutms_test_gui.main_window")

# Topics whose publish rate the GUI monitors (verified from source).
MONITORED_TOPICS = [
    ("/vehicle/velocity", "std_msgs.msg.Float32"),
    ("/vehicle/steering_angle", "std_msgs.msg.Float32"),
    ("/odometry/sbg_ekf", "nav_msgs.msg.Odometry"),
    ("/control/driving_command", "ackermann_msgs.msg.AckermannDriveStamped"),
    ("/system/av_state", "driverless_msgs.msg.AVStateStamped"),
    ("/system/ros_state", "driverless_msgs.msg.ROSStateStamped"),
]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QUTMS Driverless x EUFS Simulator")
        self.resize(1280, 800)

        # --- Core managers ------------------------------------------------
        self.ros_manager = RosManager()
        self.topic_monitor = TopicMonitor(self.ros_manager)
        self.vehicle_state = VehicleState(self.ros_manager)
        self.process_manager = ProcessManager()

        # --- Pages --------------------------------------------------------
        self.dashboard = DashboardPage()
        self.nodes_page = NodesPage()
        self.topics_page = TopicsPage()
        self.vehicle_page = VehiclePage()
        self.mission_page = MissionPage()
        self.logs_page = LogsPage()

        self.tabs = QTabWidget()
        self.tabs.addTab(self.dashboard, "Dashboard")
        self.tabs.addTab(self.nodes_page, "Nodes")
        self.tabs.addTab(self.topics_page, "Topics")
        self.tabs.addTab(self.vehicle_page, "Vehicle")
        self.tabs.addTab(self.mission_page, "Mission")
        self.tabs.addTab(self.logs_page, "Logs")

        self.setCentralWidget(self.tabs)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

        self._wire_signals()
        self._start_ros()

    # --- Signal wiring ---------------------------------------------------
    def _wire_signals(self):
        # Logs -> Logs page.
        get_log_signal().log_emitted.connect(self.logs_page.append_log)

        # ROS2 graph -> Dashboard + Nodes + Topics pages.
        self.ros_manager.ros2_status_changed.connect(self.dashboard.set_ros2_connected)
        self.ros_manager.graph_updated.connect(self._refresh_graph)

        # Topic stats -> Topics page.
        self.topic_monitor.topic_stats.connect(self._refresh_topic_stats)

        # Vehicle state -> Dashboard + Vehicle page.
        # Payload-free signals: pull a snapshot in the GUI thread rather than
        # passing Python dicts across the rclpy spin thread (PySide6 crash).
        self.vehicle_state.vehicle_updated.connect(self._refresh_vehicle)
        self.vehicle_state.control_updated.connect(self._refresh_control)
        self.vehicle_state.system_state_updated.connect(self._refresh_system)

        # Dashboard controls.
        self.dashboard.start_sim_requested.connect(self._on_start_sim)
        self.dashboard.stop_sim_requested.connect(self._on_stop_sim)
        self.dashboard.restart_sim_requested.connect(self._on_restart_sim)
        self.dashboard.emergency_stop_requested.connect(self._on_emergency_stop)
        self.dashboard.start_mission_requested.connect(self._on_start_mission)
        self.dashboard.pause_mission_requested.connect(self._on_pause_mission)
        self.dashboard.reset_mission_requested.connect(self._on_reset_mission)

        # Mission page controls.
        self.mission_page.start_mission_requested.connect(self._on_start_mission)
        self.mission_page.pause_mission_requested.connect(self._on_pause_mission)
        self.mission_page.reset_mission_requested.connect(self._on_reset_mission)
        self.mission_page.emergency_stop_requested.connect(self._on_emergency_stop)

        # Process manager -> status bar.
        self.process_manager.state_changed.connect(self._on_process_state)
        self.process_manager.stdout_line.connect(
            lambda line: logger.info("EUFS: %s", line)
        )
        self.process_manager.stderr_line.connect(
            lambda line: logger.warn("EUFS-ERR: %s", line)
        )

    # --- ROS lifecycle ---------------------------------------------------
    def _start_ros(self):
        logger.info("Starting ROS2 manager")
        self.ros_manager.start()
        if self.ros_manager.ros2_ok:
            self.topic_monitor.register_topics(MONITORED_TOPICS)
            self.topic_monitor.start()
            self.vehicle_state.start()

    # --- Snapshot pull-back (GUI thread) --------------------------------
    def _refresh_graph(self):
        graph = self.ros_manager.last_graph()
        self.dashboard.update_graph(graph)
        self.nodes_page.update_nodes(graph)
        self.topics_page.update_topics(graph)

    def _refresh_topic_stats(self):
        self.topics_page.update_topic_stats(self.topic_monitor.last_snapshot())

    def _refresh_vehicle(self):
        state = self.vehicle_state.snapshot_vehicle()
        self.dashboard.update_vehicle(state)
        self.vehicle_page.update_vehicle(state)

    def _refresh_control(self):
        control = self.vehicle_state.snapshot_control()
        self.dashboard.update_control(control)
        self.vehicle_page.update_control(control)

    def _refresh_system(self):
        system = self.vehicle_state.snapshot_system()
        self.dashboard.update_subsystem_status(system)
        self.mission_page.update_system_state(system)

    # --- Simulator process control --------------------------------------
    def _on_start_sim(self):
        if self.process_manager.is_running:
            logger.warn("Simulator already running")
            return

        mode = self.dashboard.launch_mode
        track = self.dashboard.track
        if mode == self.dashboard.MODE_YOLO_AUTONOMOUS:
            yolo_path = self.dashboard.tcone_yolo_path
            planner = self.dashboard.planner
            logger.info(
                "Starting YOLO autonomous loop (track=%s, planner=%s, tcone_yolo_path=%s)",
                track, planner, yolo_path,
            )
            self.process_manager.start(
                "ros2",
                [
                    "launch",
                    "qutms_eufs_bridge",
                    "yolo_autonomous_loop.launch.py",
                    f"track:={track}",
                    f"planner:={planner}",
                    f"tcone_yolo_path:={yolo_path}",
                ],
            )
        else:
            logger.info("Starting eufs simulator (minimal control loop, track=%s)", track)
            self.process_manager.start(
                "ros2",
                ["launch", "qutms_eufs_bridge", "minimal_control_loop.launch.py", f"track:={track}"],
            )

    def _on_stop_sim(self):
        logger.info("Stopping eufs simulator")
        self.process_manager.stop()

    def _on_restart_sim(self):
        logger.info("Restarting eufs simulator")
        self.process_manager.restart()

    def _on_process_state(self, state: str):
        self.statusBar().showMessage(f"Simulator: {state}")
        logger.info("Simulator state: %s", state)

    # --- Mission / emergency control (real ROS2 interfaces) -------------
    def _on_start_mission(self, mission_id: int):
        if not self.ros_manager.ros2_ok or self.ros_manager.node is None:
            logger.error("ROS2 not connected; cannot start mission")
            return
        logger.info("Starting mission id=%s", mission_id)
        try:
            from std_msgs.msg import UInt8
            pub = self.ros_manager.node.create_publisher(UInt8, "/system/mission_select", 10)
            msg = UInt8()
            msg.data = int(mission_id)
            pub.publish(msg)
            logger.info("Published /system/mission_select = %s", mission_id)
            self.ros_manager.node.destroy_publisher(pub)
        except Exception as exc:
            logger.error("Failed to publish mission_select: %s", exc)

    def _on_pause_mission(self):
        # No confirmed pause interface exists in the codebase.
        logger.warn("Mission pause interface NOT CONFIGURED")

    def _on_reset_mission(self):
        if not self.ros_manager.ros2_ok or self.ros_manager.node is None:
            logger.error("ROS2 not connected; cannot reset mission")
            return
        logger.info("Resetting mission")
        try:
            from driverless_msgs.msg import Reset
            pub = self.ros_manager.node.create_publisher(Reset, "/system/reset", 10)
            pub.publish(Reset(reset=True))
            logger.info("Published /system/reset")
            self.ros_manager.node.destroy_publisher(pub)
        except Exception as exc:
            logger.error("Failed to publish reset: %s", exc)

        # Also call eufs reset services if available.
        self._call_trigger_service("/system/reset_car_pos")
        self._call_trigger_service("/system/reset_cones")

    def _on_emergency_stop(self):
        logger.warn("EMERGENCY STOP requested")
        if not self.ros_manager.ros2_ok or self.ros_manager.node is None:
            logger.error("ROS2 not connected; emergency stop limited to logging")
            return
        # 1. Publish shutdown with emergency flag.
        try:
            from driverless_msgs.msg import Shutdown
            pub = self.ros_manager.node.create_publisher(Shutdown, "/system/shutdown", 10)
            pub.publish(Shutdown(emergency_shutdown=True))
            logger.info("Published /system/shutdown (emergency_shutdown=True)")
            self.ros_manager.node.destroy_publisher(pub)
        except Exception as exc:
            logger.error("Failed to publish shutdown: %s", exc)
        # 2. Publish zero-speed driving command so eufs vehicle actually stops.
        try:
            from ackermann_msgs.msg import AckermannDriveStamped
            pub = self.ros_manager.node.create_publisher(
                AckermannDriveStamped, "/control/driving_command", 10
            )
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            cmd.drive.acceleration = 0.0
            pub.publish(cmd)
            logger.info("Published zero-speed /control/driving_command")
            self.ros_manager.node.destroy_publisher(pub)
        except Exception as exc:
            logger.error("Failed to publish zero-speed command: %s", exc)

    def _call_trigger_service(self, service_name: str):
        if not self.ros_manager.ros2_ok or self.ros_manager.node is None:
            return
        try:
            from std_srvs.srv import Trigger
            client = self.ros_manager.node.create_client(Trigger, service_name)
            if not client.wait_for_service(timeout_sec=2.0):
                logger.warn("Service %s not available", service_name)
                self.ros_manager.node.destroy_client(client)
                return
            future = client.call_async(Trigger.Request())
            # Block briefly; this runs on a signal slot (GUI thread) so keep it short.
            import time
            deadline = time.time() + 2.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.05)
            if future.done():
                resp = future.result()
                logger.info("Service %s response: success=%s", service_name, resp.success)
            else:
                logger.warn("Service %s timed out", service_name)
            self.ros_manager.node.destroy_client(client)
        except Exception as exc:
            logger.error("Service %s call failed: %s", service_name, exc)

    # --- Cleanup ---------------------------------------------------------
    def closeEvent(self, event):
        logger.info("Shutting down GUI")
        self.process_manager.stop()
        self.topic_monitor.stop()
        self.vehicle_state.stop()
        self.ros_manager.stop()
        super().closeEvent(event)