"""Topic frequency and freshness monitor.

Subscribes to a configurable set of known topics (name + message type) and
reports publish rate and staleness to the GUI through Qt signals.

Only topics whose message type is known at build time are monitored. The
graph-wide topic list comes from RosManager, not from this class.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict

from PySide6.QtCore import QObject, Signal
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Publishers in this stack use best-effort; a reliable subscription would
# never match them (and would report 0 Hz for every topic).
_MONITOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui.topic_monitor")

# Stale threshold in seconds. If no message arrives within this window the
# topic is reported as STALE.
STALE_THRESHOLD_SEC = 2.0


@dataclass
class TopicStats:
    name: str
    msg_type: str
    count: int = 0
    last_time: float = 0.0
    rate_hz: float = 0.0
    status: str = "UNKNOWN"  # OK | STALE | NO DATA | UNKNOWN
    window_start: float = field(default_factory=time.time)
    window_count: int = 0


class TopicMonitor(QObject):
    """Monitors a fixed list of known topics for publish rate.

    Signals:
        topic_stats(dict): per-topic statistics keyed by topic name.
    """

    # Payload-free: see ros_manager/vehicle_state for the PySide6 crash.
    topic_stats = Signal()

    def __init__(self, ros_manager):
        super().__init__()
        self._ros = ros_manager
        self._subscriptions = {}  # topic_name -> rclpy subscription
        self._stats_lock = threading.Lock()
        self._last_snapshot = {}
        self._stats: Dict[str, TopicStats] = {}
        self._lock = threading.Lock()
        self._report_thread = None
        self._running = False
        self._stop_event = threading.Event()

    def register_topics(self, topic_specs):
        """Register topics to monitor.

        Args:
            topic_specs: list of (topic_name, msg_type_module_path) tuples,
                e.g. [("/vehicle/velocity", "std_msgs.msg.Float32")]
        """
        with self._lock:
            for name, type_path in topic_specs:
                if name not in self._stats:
                    self._stats[name] = TopicStats(name=name, msg_type=type_path)

    def start(self):
        if not self._ros.ros2_ok or self._ros.node is None:
            logger.warn("ROS2 not available; topic monitor not started")
            return

        for name, stats in self._stats.items():
            try:
                msg_type = self._import_type(stats.msg_type)
                sub = self._ros.node.create_subscription(
                    msg_type, name, self._make_callback(name), _MONITOR_QOS
                )
                self._subscriptions[name] = sub
                logger.info("Monitoring topic %s (%s)", name, stats.msg_type)
            except Exception as exc:
                logger.error("Failed to subscribe to %s: %s", name, exc)

        self._running = True
        self._stop_event.clear()
        self._report_thread = threading.Thread(target=self._report_loop, daemon=True)
        self._report_thread.start()

    def stop(self):
        self._running = False
        self._stop_event.set()
        if self._report_thread is not None:
            self._report_thread.join(timeout=2.0)
            self._report_thread = None
        for name, sub in self._subscriptions.items():
            try:
                self._ros.node.destroy_subscription(sub)
            except Exception:
                pass
        self._subscriptions.clear()

    def _make_callback(self, topic_name):
        def callback(msg):
            with self._lock:
                stats = self._stats.get(topic_name)
                if stats is None:
                    return
                stats.count += 1
                stats.window_count += 1
                stats.last_time = time.time()

        return callback

    def _report_loop(self):
        while self._running:
            self._stop_event.wait(1.0)
            if not self._running:
                break
            now = time.time()
            snapshot = {}
            with self._lock:
                for name, stats in self._stats.items():
                    if stats.count == 0:
                        stats.status = "NO DATA"
                    else:
                        age = now - stats.last_time
                        if age > STALE_THRESHOLD_SEC:
                            stats.status = "STALE"
                        else:
                            stats.status = "OK"
                        # Simple 1-second window rate.
                        stats.rate_hz = stats.window_count
                        stats.window_count = 0
                    snapshot[name] = {
                        "name": stats.name,
                        "msg_type": stats.msg_type,
                        "rate_hz": round(stats.rate_hz, 2),
                        "status": stats.status,
                        "last_update_ago": round(now - stats.last_time, 2)
                        if stats.last_time > 0
                        else None,
                    }
            with self._stats_lock:
                self._last_snapshot = snapshot
            self.topic_stats.emit()

    def last_snapshot(self) -> dict:
        """Most recent per-topic statistics (read from the GUI thread)."""
        with self._stats_lock:
            return dict(self._last_snapshot)

    @staticmethod
    def _import_type(dotted_path: str):
        """Import a message type from a dotted path like 'std_msgs.msg.Float32'."""
        parts = dotted_path.rsplit(".", 1)
        if len(parts) != 2:
            raise ValueError(f"Invalid type path: {dotted_path}")
        module_path, class_name = parts
        module = __import__(module_path, fromlist=[class_name])
        return getattr(module, class_name)