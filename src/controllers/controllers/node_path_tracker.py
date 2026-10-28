"""Path tracker: adaptive-lookahead Pure Pursuit with curvature feedforward.

Consumes `driverless_msgs/PathWithSpeeds` (map frame), vehicle velocity and
steering feedback, and produces `ackermann_msgs/AckermannDriveStamped` on
`/control/driving_command` (steering in DEGREES, ±90° = full lock in the
EUFS RaceCarPlugin convention).

Pipeline per cycle:
    1. pose from TF (map -> base_link)
    2. nearest path point, adaptive lookahead Ld = clamp(kv*v + kl, min, max)
    3. pure pursuit: delta = atan2(2*L*sin(alpha), Ld)
    4. curvature feedforward: delta += ff_gain * atan(wheelbase * kappa)
    5. optional slip-angle compensation (vy_topic non-empty enables it)
    6. steering PID (anti-windup, derivative-on-measurement, rate limit)
    7. longitudinal: target speed at the lookahead point, ramp-limited
       (the EUFS plugin tracks speed internally in velocity mode)

On path staleness the tracker itself commands zero speed (the plugin would
otherwise hold the last command); the safety watchdog is the second layer.
"""

from __future__ import annotations

import math
import time

import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from driverless_msgs.msg import PathWithSpeeds, Reset, Shutdown
from driverless_common.common import QOS_COMMAND, QOS_LATEST, wrap_to_pi, yaw
from std_msgs.msg import Float32


class PathTracker(Node):
    def __init__(self) -> None:
        super().__init__("path_tracker_node")

        # frames / topics
        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("path_topic", "/planning/path_with_speeds")
        self.declare_parameter("velocity_topic", "/vehicle/velocity")
        self.declare_parameter("steering_fb_topic", "/vehicle/steering_angle")
        self.declare_parameter("vy_topic", "")  # empty -> slip compensation off
        self.declare_parameter("cmd_topic", "/control/driving_command")
        self.declare_parameter("shutdown_topic", "/system/shutdown")
        self.declare_parameter("reset_topic", "/system/reset")

        # control loop
        self.declare_parameter("control_rate", 20.0)  # [Hz]
        self.declare_parameter("path_timeout_sec", 0.5)  # tracker-level staleness

        # lookahead
        self.declare_parameter("lookahead_kv", 0.55)
        self.declare_parameter("lookahead_kl", 1.8)
        self.declare_parameter("min_lookahead", 2.0)
        self.declare_parameter("max_lookahead", 8.0)
        # If the first and last path points are this close [m], treat the path
        # as a closed loop and wrap the lookahead index around the seam.
        self.declare_parameter("closed_path_gap_tol", 1.0)

        # vehicle model
        self.declare_parameter("wheelbase", 1.5)

        # steering
        # The EUFS plugin maps a 90 deg command onto the physical steering lock
        # (0.28 rad / 16 deg, from configDry.yaml input_ranges.steering), i.e.
        # cmd_deg = phys_deg * 90/16. Pure pursuit works in a physical wheel
        # angle, so commands are scaled up by steer_cmd_gain and the steering
        # feedback is scaled back down: without this the car is commanded to
        # ~1/5.6 of the angle it needs and runs wide on every corner.
        self.declare_parameter("steer_cmd_gain", 5.625)  # [cmd deg / phys deg]
        self.declare_parameter("max_steering_cmd_deg", 90.0)  # command-space limit
        self.declare_parameter("curvature_ff_gain", 1.0)
        self.declare_parameter("slip_gain", 0.5)
        self.declare_parameter("steer_kp", 1.0)
        self.declare_parameter("steer_ki", 0.0)
        self.declare_parameter("steer_kd", 0.0)
        self.declare_parameter("steer_rate_frac", 0.5)  # max output change per cycle, frac of max

        # longitudinal
        self.declare_parameter("speed_ramp_accel", 2.5)  # [m/s^2] on the commanded speed
        self.declare_parameter("speed_ramp_brake", 4.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.map_frame = g("map_frame")
        self.base_frame = g("base_frame")
        self.path_timeout = float(g("path_timeout_sec"))
        self.kv = float(g("lookahead_kv"))
        self.kl = float(g("lookahead_kl"))
        self.min_lookahead = float(g("min_lookahead"))
        self.max_lookahead = float(g("max_lookahead"))
        self.closed_gap_tol = float(g("closed_path_gap_tol"))
        self.wheelbase = float(g("wheelbase"))
        self.steer_cmd_gain = float(g("steer_cmd_gain"))
        self.max_cmd_deg = float(g("max_steering_cmd_deg"))
        self.max_steer_rad = math.radians(self.max_cmd_deg / self.steer_cmd_gain)
        self.ff_gain = float(g("curvature_ff_gain"))
        self.slip_gain = float(g("slip_gain"))
        self.kp = float(g("steer_kp"))
        self.ki = float(g("steer_ki"))
        self.kd = float(g("steer_kd"))
        self.steer_rate_frac = max(0.01, float(g("steer_rate_frac")))
        self.ramp_accel = float(g("speed_ramp_accel"))
        self.ramp_brake = float(g("speed_ramp_brake"))

        # state
        self.path_points: np.ndarray | None = None  # (N, 2)
        self.path_speeds: np.ndarray | None = None  # (N,)
        self.path_curvatures: np.ndarray | None = None  # (N,)
        self.path_header = None
        self.path_recv_mono = 0.0
        self.velocity = 0.0
        self.vy = 0.0
        self.steer_meas_rad = 0.0
        # PID state
        self.integral = 0.0
        self.last_steer_meas = None
        self.last_cmd_rad = 0.0
        self.cmd_speed = 0.0
        self.dt = 1.0 / float(g("control_rate"))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, g("cmd_topic"), QOS_COMMAND
        )
        self.create_subscription(
            PathWithSpeeds, g("path_topic"), self.path_callback, QOS_LATEST
        )
        self.create_subscription(Float32, g("velocity_topic"), self.velocity_callback, QOS_LATEST)
        self.create_subscription(
            Float32, g("steering_fb_topic"), self.steering_callback, QOS_LATEST
        )
        vy_topic = g("vy_topic")
        self.slip_enabled = bool(vy_topic)
        if self.slip_enabled:
            self.create_subscription(Float32, vy_topic, self.vy_callback, QOS_LATEST)

        # emergency-stop latch shared with the safety watchdog: both nodes
        # halt on /system/shutdown so the GUI's one-shot stop is not raced
        # by this 20 Hz loop; /system/reset releases the latch
        self.shutdown_latched = False
        self.create_subscription(Shutdown, g("shutdown_topic"), self.shutdown_callback, QOS_LATEST)
        self.create_subscription(Reset, g("reset_topic"), self.reset_callback, QOS_LATEST)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f"---Path tracker initialised ({float(g('control_rate')):.0f}Hz, "
            f"Ld={self.min_lookahead}-{self.max_lookahead}m, "
            f"slip={'on' if self.slip_enabled else 'off'})---"
        )

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #
    def path_callback(self, msg: PathWithSpeeds) -> None:
        if not msg.poses:
            return
        self.path_points = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=np.float64
        )
        self.path_speeds = np.array(msg.target_speeds, dtype=np.float64)
        self.path_curvatures = np.array(msg.curvatures, dtype=np.float64)
        self.path_header = msg.header
        self.path_recv_mono = time.monotonic()

    def velocity_callback(self, msg: Float32) -> None:
        self.velocity = float(msg.data)

    def steering_callback(self, msg: Float32) -> None:
        # Feedback is published by the plugin in command degrees; convert it
        # back to a physical wheel angle so the PID acts on one unit system.
        self.steer_meas_rad = math.radians(float(msg.data) / self.steer_cmd_gain)

    def vy_callback(self, msg: Float32) -> None:
        self.vy = float(msg.data)

    def shutdown_callback(self, msg: Shutdown) -> None:
        if msg.emergency_shutdown or msg.finished_engage_ebs:
            if not self.shutdown_latched:
                self.get_logger().error("EMERGENCY SHUTDOWN received — halting commands.")
            self.shutdown_latched = True
            self.cmd_speed = 0.0

    def reset_callback(self, msg: Reset) -> None:
        if msg.reset and self.shutdown_latched:
            self.shutdown_latched = False
            self.get_logger().warn("Shutdown latch cleared by /system/reset.")

    # ------------------------------------------------------------------ #
    # control
    # ------------------------------------------------------------------ #
    def control_loop(self) -> None:
        # latched emergency stop: hold zero commands until /system/reset
        if self.shutdown_latched:
            self.publish_command(steering_deg=0.0, speed=0.0, stopping=True)
            return

        # tracker-level failsafe: the EUFS plugin holds the last speed command,
        # so on a stale path we actively command zero rather than go silent
        if self.path_points is None or (time.monotonic() - self.path_recv_mono) > self.path_timeout:
            self.publish_command(steering_deg=0.0, speed=0.0, stopping=True)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().warn("TF unavailable", throttle_duration_sec=2.0)
            self.publish_command(steering_deg=0.0, speed=0.0, stopping=True)
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        heading = yaw(tf.transform.rotation)

        # nearest path point
        points = self.path_points
        d_sq = np.sum((points - np.array([x, y])) ** 2, axis=1)
        nearest = int(np.argmin(d_sq))

        # A closed-loop path is published in one fixed winding order, but the
        # car can be heading either way along it (e.g. after recovery). Walking
        # the array forward from the nearest point would then put the lookahead
        # point behind the car, and pure pursuit would hold full lock forever.
        # An OPEN path (partial cone map) must only be followed forward: the
        # car often sits past its last point, and walking backwards there makes
        # the controller attempt a U-turn and leave the track.
        n = len(points)
        # Closed loops wrap around their seam. Without wrapping, the lookahead
        # walk stops at the last sample and the target collapses onto the car
        # whenever the nearest point is near the seam, causing a steering jerk.
        closed_path = n >= 4 and float(
            np.linalg.norm(points[0] - points[-1])
        ) <= self.closed_gap_tol
        if closed_path:
            fwd = (nearest + 1) % n
            bwd = (nearest - 1) % n
            def _ahead(idx: int) -> float:
                dx, dy = points[idx] - np.array([x, y])
                return math.cos(heading) * dx + math.sin(heading) * dy
            step = 1 if _ahead(fwd) >= _ahead(bwd) else -1
        else:
            step = 1

        # adaptive lookahead target: walk arc length from the nearest point
        lookahead = min(
            self.max_lookahead, max(self.min_lookahead, self.kv * self.velocity + self.kl)
        )
        target_idx = nearest
        accumulated = 0.0
        i = nearest
        max_steps = n if closed_path else n - 1
        for _ in range(max_steps):
            nxt = (i + step) % n if closed_path else i + step
            if not closed_path and not (0 <= nxt < n):
                break
            accumulated += float(np.linalg.norm(points[nxt] - points[i]))
            i = nxt
            target_idx = i
            if accumulated >= lookahead:
                break

        # target point into base frame
        tx, ty = points[target_idx]
        dx, dy = tx - x, ty - y
        x_b = math.cos(heading) * dx + math.sin(heading) * dy
        y_b = -math.sin(heading) * dx + math.cos(heading) * dy

        # pure pursuit + curvature feedforward
        alpha = math.atan2(y_b, x_b)
        delta = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead)
        if self.ff_gain > 0.0 and self.path_curvatures is not None:
            kappa = float(self.path_curvatures[target_idx])
            # Curvature is defined in the path's forward winding; driving the
            # path backwards flips the required steering sign.
            if step < 0:
                kappa = -kappa
            delta += self.ff_gain * math.atan(self.wheelbase * kappa)

        # slip-angle compensation (optional; needs a lateral velocity source)
        if self.slip_enabled and self.velocity > 1.0:
            delta -= self.slip_gain * math.atan2(self.vy, max(self.velocity, 0.5))

        delta = self.steering_pid(delta)

        # longitudinal: target speed at the lookahead point, ramp-limited
        v_target = float(self.path_speeds[target_idx]) if self.path_speeds is not None else 0.0
        if not math.isfinite(v_target) or v_target < 0.0:
            v_target = 0.0
        delta_v = v_target - self.cmd_speed
        max_up = self.ramp_accel * self.dt
        max_down = self.ramp_brake * self.dt
        self.cmd_speed += max(-max_down, min(max_up, delta_v))
        self.cmd_speed = max(0.0, self.cmd_speed)

        self.publish_command(
            steering_deg=math.degrees(delta) * self.steer_cmd_gain,
            speed=self.cmd_speed,
            stopping=self.cmd_speed < 0.05,
        )

    def steering_pid(self, delta_target: float) -> float:
        """PID on (target - measured) steering with anti-windup and rate limit.

        The measured steering is added as a bias (same structure as the HUAT
        controller), so with kp=1, ki=kd=0 the output equals the target and
        higher kp only sharpens the response to tracking error.
        """
        error = wrap_to_pi(delta_target - self.steer_meas_rad)

        if abs(error) <= self.max_steer_rad:
            self.integral += error
            max_integral = self.max_steer_rad / max(self.ki, 1e-6)
            self.integral = max(-max_integral, min(self.integral, max_integral))
        else:
            self.integral *= 0.9  # decay while saturated to avoid recovery oscillation

        deriv = 0.0
        if self.kd > 0.0:
            if self.last_steer_meas is not None:
                deriv = -(self.steer_meas_rad - self.last_steer_meas) / self.dt
            self.last_steer_meas = self.steer_meas_rad

        out = self.steer_meas_rad + self.kp * error + self.ki * self.integral + self.kd * deriv
        out = max(-self.max_steer_rad, min(self.max_steer_rad, out))

        # output rate limit per cycle
        max_delta = self.max_steer_rad * self.steer_rate_frac
        out = max(self.last_cmd_rad - max_delta, min(self.last_cmd_rad + max_delta, out))
        self.last_cmd_rad = out
        return out

    # ------------------------------------------------------------------ #
    def publish_command(self, steering_deg: float, speed: float, stopping: bool = False) -> None:
        steering_deg = max(-self.max_cmd_deg, min(self.max_cmd_deg, steering_deg))
        if stopping:
            self.cmd_speed = 0.0

        msg = AckermannDriveStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_deg  # degrees (EUFS plugin convention)
        msg.drive.speed = speed

        self.cmd_pub.publish(msg)

        if self.path_header is not None and not stopping:
            e2e_ms = (
                self.get_clock().now().nanoseconds / 1e6
                - self.path_header.stamp.sec * 1e3
                - self.path_header.stamp.nanosec / 1e6
            )
            self.get_logger().info(
                f"v={self.velocity:.2f} cmd_v={speed:.2f} steer={steering_deg:+.1f}deg "
                f"e2e={e2e_ms:.0f}ms",
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
