"""ROS2 graph manager.

Runs rclpy in a background thread and exposes node/topic/service/action
information to the GUI through Qt signals. Never blocks the GUI thread.
"""

import threading
import time

from PySide6.QtCore import QObject, Signal

try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui.ros_manager")


class RosManager(QObject):
    """Manages rclpy lifecycle and polls the ROS2 graph.

    Signals:
        ros2_status_changed(bool): ROS2 available / connected.
        graph_updated(dict): latest node/topic/service/action counts and names.
    """

    ros2_status_changed = Signal(bool)
    # Payload-free: Python objects must not cross the spin thread into Qt
    # (see vehicle_state for the PySide6 refcount crash this avoids).
    graph_updated = Signal()

    # Polling interval (seconds) for graph info.
    POLL_INTERVAL = 2.0

    def __init__(self):
        super().__init__()
        self._node = None
        self._spin_thread = None
        self._poll_thread = None
        self._running = False
        self._ros2_ok = False
        self._stop_event = threading.Event()
        self._last_graph = {}

    def start(self):
        """Initialise rclpy, create a hidden node, and start spin + poll threads."""
        if not ROS2_AVAILABLE:
            logger.error("rclpy not available; ROS2 status will be UNKNOWN")
            self._ros2_ok = False
            self.ros2_status_changed.emit(False)
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("qutms_test_gui_ros_manager")
            self._ros2_ok = True
            self.ros2_status_changed.emit(True)
            logger.info("ROS2 connected")
        except Exception as exc:
            logger.error("Failed to initialise rclpy: %s", exc)
            self._ros2_ok = False
            self.ros2_status_changed.emit(False)
            return

        self._running = True
        self._stop_event.clear()

        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop(self):
        self._running = False
        self._stop_event.set()
        # Wait for spin/poll threads to exit before tearing down rclpy so they
        # never operate on a destroyed node / shutdown context.
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self._ros2_ok = False
        self.ros2_status_changed.emit(False)
        logger.info("ROS2 disconnected")

    @property
    def ros2_ok(self) -> bool:
        return self._ros2_ok

    @property
    def node(self):
        return self._node

    def last_graph(self) -> dict:
        """Snapshot of the most recently polled ROS graph (GUI thread)."""
        return self._last_graph

    def _spin_loop(self):
        while self._running and self._node is not None:
            # rclpy may already be shut down (e.g. GUI window closed while the
            # context was still alive). spin_once on an invalid context raises
            # forever, so leave instead of flooding the log at 2 Hz.
            if not rclpy.ok():
                break
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as exc:
                logger.error("rclpy spin error: %s", exc)
                if not rclpy.ok():
                    break
                time.sleep(0.5)

    def _poll_loop(self):
        while self._running and self._node is not None:
            if not rclpy.ok():
                break
            try:
                self._last_graph = self._collect_graph()
                self.graph_updated.emit()
            except Exception as exc:
                logger.error("graph poll error: %s", exc)
            self._stop_event.wait(self.POLL_INTERVAL)

    def _collect_graph(self) -> dict:
        node_names = self._node.get_node_names()
        # get_node_names returns names without namespace prefix; combine with namespaces.
        names_and_namespaces = self._node.get_node_names_and_namespaces()
        nodes = [
            {"name": ns + name if ns != "/" else name, "namespace": ns}
            for name, ns in names_and_namespaces
        ]

        topics = []
        for topic_name, type_list in self._node.get_topic_names_and_types():
            publishers = self._node.count_publishers(topic_name)
            subscribers = self._node.count_subscribers(topic_name)
            topics.append(
                {
                    "name": topic_name,
                    "types": type_list,
                    "publishers": publishers,
                    "subscribers": subscribers,
                }
            )

        services = [
            {"name": name, "types": types}
            for name, types in self._node.get_service_names_and_types()
        ]

        actions = []
        try:
            for name, types in self._node.get_action_names_and_types():
                actions.append({"name": name, "types": types})
        except Exception:
            # Older rclpy may not expose action graph info.
            pass

        return {
            "node_count": len(nodes),
            "nodes": nodes,
            "topic_count": len(topics),
            "topics": topics,
            "service_count": len(services),
            "services": services,
            "action_count": len(actions),
            "actions": actions,
        }