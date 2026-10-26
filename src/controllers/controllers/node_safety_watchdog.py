"""Safety watchdog: second-layer failsafe behind the path tracker.

Monitors the freshness of `/planning/path_with_speeds` using the WALL
clock (immune to /use_sim_time jumps and pauses during sim replay) plus
TF availability. When either is stale beyond the timeout, publishes a
stop command (zero speed, straight steering, strong deceleration) at
10 Hz until inputs recover.

Also LATCHES on `/system/shutdown` (emergency_shutdown or
finished_engage_ebs) so the test GUI's EMERGENCY STOP button actually
holds: a one-shot zero-speed command from the GUI would otherwise be
overridden by the 20 Hz tracker within one cycle. The latch persists
(ignore fresh paths) until `/system/reset` clears it.

The path tracker already commands zero speed on its own (shorter)
timeout; this node catches total failures — planner crash, tracker
crash, TF loss — where nothing else is publishing on the command topic.
"""

from __future__ import annotations

import time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathWithSpeeds, Reset, Shutdown
from driverless_common.common import QOS_COMMAND, QOS_LATEST


class SafetyWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_watchdog_node")

        self.declare_parameter("path_topic", "/planning/path_with_speeds")
        self.declare_parameter("cmd_topic", "/control/driving_command")
        self.declare_parameter("shutdown_topic", "/system/shutdown")
        self.declare_parameter("reset_topic", "/system/reset")
        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("timeout_sec", 2.0)  # wall-clock staleness limit
        self.declare_parameter("stop_rate", 10.0)   # [Hz] stop command repetition
        self.declare_parameter("stop_accel", -5.0)  # m/s^2 advertised on the stop frame

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.timeout = float(g("timeout_sec"))
        self.stop_accel = float(g("stop_accel"))
        self.map_frame = g("map_frame")
        self.base_frame = g("base_frame")

        self.last_path_mono: float | None = None
        self.stop_active = False
        self.shutdown_latched = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, g("cmd_topic"), QOS_COMMAND)
        self.create_subscription(
            PathWithSpeeds, g("path_topic"), self.path_callback, QOS_LATEST
        )
        self.create_subscription(Shutdown, g("shutdown_topic"), self.shutdown_callback, QOS_LATEST)
        self.create_subscription(Reset, g("reset_topic"), self.reset_callback, QOS_LATEST)
        self.create_timer(1.0 / float(g("stop_rate")), self.check_loop)

        self.get_logger().info(
            f"---Safety watchdog initialised (path timeout {self.timeout:.1f}s wall-clock)---"
        )

    def path_callback(self, msg: PathWithSpeeds) -> None:
        self.last_path_mono = time.monotonic()
        if self.stop_active and not self.shutdown_latched:
            self.stop_active = False
            self.get_logger().warn("Watchdog cleared: fresh path received, releasing stop.")

    def shutdown_callback(self, msg: Shutdown) -> None:
        if msg.emergency_shutdown or msg.finished_engage_ebs:
            if not self.shutdown_latched:
                self.get_logger().error(
                    "EMERGENCY SHUTDOWN received — latching stop (reset via /system/reset)."
                )
            self.shutdown_latched = True

    def reset_callback(self, msg: Reset) -> None:
        if msg.reset and self.shutdown_latched:
            self.shutdown_latched = False
            self.get_logger().warn("Watchdog shutdown latch cleared by /system/reset.")

    def check_loop(self) -> None:
        if self.last_path_mono is None:
            return  # nothing ever published yet; tracker/path silence handled once driving starts

        # latched emergency shutdown takes priority over fresh paths
        if self.shutdown_latched:
            self.publish_stop("emergency shutdown latched")
            return

        path_age = time.monotonic() - self.last_path_mono

        tf_ok = True
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            tf_ok = False

        if path_age <= self.timeout and tf_ok:
            return

        reason = f"path stale {path_age:.2f}s" if path_age > self.timeout else "TF unavailable"
        if not self.stop_active:
            self.stop_active = True
            self.get_logger().error(f"Watchdog triggered: {reason}. Commanding stop.")
        self.publish_stop(reason)

    def publish_stop(self, reason: str) -> None:
        msg = AckermannDriveStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        msg.drive.acceleration = self.stop_accel
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"stop command active ({reason})", throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
