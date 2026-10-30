"""Skidpad planner: template fit -> analytic path -> windowed PathWithSpeeds.

The regular planners cannot drive a figure-eight (urinay forbids self
intersections, ft's left/right matching fails at the crossing). This node
fits the two-circle skidpad template from ``/mapping/cone_map``, generates
the analytic path once, freezes the template, and publishes only a short
window ahead of a monotonic progress value so the existing Pure Pursuit
tracker's global nearest-point remains unambiguous.

It never publishes a path before the fit passes every quality gate, and it
does not modify the tracker, urinay, ft or cone_mapping.
"""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from driverless_common.common import QOS_LATEST, yaw as quat_yaw
from driverless_msgs.msg import ConeDetectionStamped, PathWithSpeeds, Reset, Shutdown
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from planners.skidpad_fitting import FitParams, fit_skidpad, select_boundary_cones
from planners.skidpad_geometry import (
    SEGMENT_LAP,
    SkidpadPath,
    compute_skidpad_speeds,
    generate_skidpad_path,
    validate_skidpad_path,
)
from planners.skidpad_progress import (
    STATE_DRIVING,
    STATE_FAULT,
    STATE_FINISHED,
    STATE_SHUTDOWN,
    STATE_WAITING,
    ProgressConfig,
    SkidpadProgress,
)
from planners.speed_profile import SpeedProfileParams

_TF_LATEST = Time()
_LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _as_float_list(value) -> List[float]:
    return [float(v) for v in value]


class SkidpadPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("skidpad_planner_node")

        # ---- I/O --------------------------------------------------------
        self.declare_parameter("cone_map_topic", "/mapping/cone_map")
        self.declare_parameter("path_topic", "/planning/path_with_speeds")
        self.declare_parameter("full_path_topic", "planning/skidpad_full_path")
        self.declare_parameter("state_topic", "planning/skidpad_state")
        self.declare_parameter("marker_topic", "planning/skidpad_markers")
        self.declare_parameter("shutdown_topic", "/system/shutdown")
        self.declare_parameter("reset_topic", "/system/reset")
        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_link")

        # ---- geometry / publishing -------------------------------------
        self.declare_parameter("ds", 0.25)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("fit_rate", 5.0)
        self.declare_parameter("window_len", 18.0)
        self.declare_parameter("progress_search_back", 1.0)
        self.declare_parameter("progress_search_ahead", 8.0)
        self.declare_parameter("off_path_dist", 2.0)
        self.declare_parameter("off_path_time", 1.0)

        # ---- fit gates (spec §6; do not relax) --------------------------
        self.declare_parameter("fit_min_cones_total", 40)
        self.declare_parameter("fit_min_cones_per_circle", 16)
        self.declare_parameter("fit_rms_max", 0.35)
        self.declare_parameter("fit_p90_max", 0.5)
        self.declare_parameter("fit_centre_dist_range", [16.0, 20.5])
        self.declare_parameter("fit_radius_range", [7.5, 11.0])
        self.declare_parameter("fit_ring_width_range", [2.4, 3.6])
        self.declare_parameter("fit_freeze_updates", 5)
        self.declare_parameter("fit_freeze_drift", 0.15)
        self.declare_parameter("fit_outlier_dist", 0.6)
        self.declare_parameter("fit_max_outlier_frac", 0.20)
        self.declare_parameter("fit_max_radius_diff", 0.5)
        self.declare_parameter("fit_max_centre_radius_mismatch", 0.5)
        self.declare_parameter("fit_max_alignment_deg", 15.0)
        self.declare_parameter("fit_entry_along_range", [3.0, 30.0])
        self.declare_parameter("fit_entry_lateral_max", 2.5)

        # ---- path / speed profile --------------------------------------
        self.declare_parameter("entry_len", 0.0)  # 0 = auto
        self.declare_parameter("entry_len_min", 10.0)
        self.declare_parameter("entry_len_max", 25.0)
        self.declare_parameter("exit_len", 20.0)
        self.declare_parameter("entry_heading_deg", "")
        self.declare_parameter("speed_cap", 2.0)  # first-test value, not a vehicle limit
        self.declare_parameter("max_lateral_acc", 5.0)
        self.declare_parameter("max_accel", 2.5)
        self.declare_parameter("max_brake", 4.0)
        self.declare_parameter("min_speed", 2.0)
        self.declare_parameter("curvature_lookahead_m", 5.0)
        self.declare_parameter("curvature_smoothing_window", 3)
        self.declare_parameter("stop_len", 4.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.map_frame = str(g("map_frame"))
        self.base_frame = str(g("base_frame"))
        self.cone_map_topic = str(g("cone_map_topic"))
        self.ds = float(g("ds"))
        self.publish_rate = max(1.0, float(g("publish_rate")))
        self.fit_rate = max(0.5, float(g("fit_rate")))
        self.window_len = float(g("window_len"))
        self.entry_len_param = float(g("entry_len"))
        self.entry_len_min = float(g("entry_len_min"))
        self.entry_len_max = float(g("entry_len_max"))
        self.exit_len = float(g("exit_len"))
        self.entry_heading_deg = str(g("entry_heading_deg")).strip()
        self.speed_cap = float(g("speed_cap"))
        self.curvature_smoothing = int(g("curvature_smoothing_window"))
        self.stop_len = float(g("stop_len"))
        self.freeze_updates = max(1, int(g("fit_freeze_updates")))
        self.freeze_drift = float(g("fit_freeze_drift"))

        self.fit_params = FitParams(
            min_cones_total=int(g("fit_min_cones_total")),
            min_cones_per_circle=int(g("fit_min_cones_per_circle")),
            rms_max=float(g("fit_rms_max")),
            p90_max=float(g("fit_p90_max")),
            centre_dist_range=tuple(_as_float_list(g("fit_centre_dist_range"))),
            radius_range=tuple(_as_float_list(g("fit_radius_range"))),
            ring_width_range=tuple(_as_float_list(g("fit_ring_width_range"))),
            outlier_dist=float(g("fit_outlier_dist")),
            max_outlier_frac=float(g("fit_max_outlier_frac")),
            max_radius_diff=float(g("fit_max_radius_diff")),
            max_centre_radius_mismatch=float(g("fit_max_centre_radius_mismatch")),
            max_alignment_deg=float(g("fit_max_alignment_deg")),
            entry_along_range=tuple(_as_float_list(g("fit_entry_along_range"))),
            entry_lateral_max=float(g("fit_entry_lateral_max")),
        )

        self.speed_params = SpeedProfileParams(
            speed_cap=self.speed_cap,
            max_lateral_acc=float(g("max_lateral_acc")),
            max_accel=float(g("max_accel")),
            max_brake=float(g("max_brake")),
            min_speed=float(g("min_speed")),
            curvature_lookahead_m=float(g("curvature_lookahead_m")),
            decel_to_stop_at_end=True,
        )

        # ---- state ------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._cones: Optional[np.ndarray] = None
        self._colors: Optional[np.ndarray] = None
        self._map_seq = 0
        self._last_fit_seq = -1
        self._last_fit_mono = 0.0

        self.entry_pose: Optional[np.ndarray] = None
        self.entry_heading: Optional[np.ndarray] = None

        self.fit = None                 # type: ignore[assignment]
        self.fit_diag: dict = {"reasons": [], "warnings": []}
        self.freeze_streak = 0
        self._last_centres: Optional[Tuple[np.ndarray, np.ndarray]] = None

        self.path: Optional[SkidpadPath] = None
        self.speeds: Optional[np.ndarray] = None
        self.curvatures: Optional[np.ndarray] = None
        self.progress: Optional[SkidpadProgress] = None
        self.state = STATE_WAITING
        self.frozen = False
        self.shutdown_latched = False
        self._finished_logged = False
        self._fault_logged = False
        self._last_marker_mono = 0.0
        self._last_wait_log = 0.0

        # ---- ROS I/O ----------------------------------------------------
        self.create_subscription(
            ConeDetectionStamped, self.cone_map_topic, self._cone_callback, QOS_LATEST
        )
        self.create_subscription(Shutdown, str(g("shutdown_topic")), self._shutdown_cb, QOS_LATEST)
        self.create_subscription(Reset, str(g("reset_topic")), self._reset_cb, QOS_LATEST)

        self.path_pub = self.create_publisher(PathWithSpeeds, str(g("path_topic")), QOS_LATEST)
        self.full_path_pub = self.create_publisher(Path, str(g("full_path_topic")), _LATCHED_QOS)
        self.state_pub = self.create_publisher(String, str(g("state_topic")), QOS_LATEST)
        self.marker_pub = self.create_publisher(MarkerArray, str(g("marker_topic")), 1)

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"---Skidpad planner ready: {self.cone_map_topic} -> "
            f"{g('path_topic')} (window {self.window_len:.1f} m)---"
        )

    # ------------------------------------------------------------------ #
    # inputs
    # ------------------------------------------------------------------ #
    def _cone_callback(self, msg: ConeDetectionStamped) -> None:
        if not msg.cones:
            self._cones = None
            self._colors = None
            return
        self._cones = np.array(
            [[c.location.x, c.location.y] for c in msg.cones], dtype=float
        )
        self._colors = np.array([int(c.color) for c in msg.cones], dtype=int)
        self._map_seq += 1

    def _lookup_pose(self) -> Optional[Tuple[np.ndarray, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, _TF_LATEST
            )
        except TransformException:
            self.get_logger().debug(
                f"TF {self.map_frame}<-{self.base_frame} unavailable",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y], dtype=float), float(quat_yaw(tf.transform.rotation))

    def _ensure_entry_frame(self) -> bool:
        if self.entry_heading is not None:
            return True
        pose = self._lookup_pose()
        if self.entry_heading_deg:
            try:
                heading = math.radians(float(self.entry_heading_deg))
            except ValueError:
                self.get_logger().error("entry_heading_deg is not a number")
                return False
            self.entry_heading = np.array([math.cos(heading), math.sin(heading)])
            if pose is not None:
                self.entry_pose = pose[0]
            return True
        if pose is None:
            return False
        self.entry_pose, yaw = pose
        self.entry_heading = np.array([math.cos(yaw), math.sin(yaw)])
        self.get_logger().info(
            f"skidpad entry frame: pose=({self.entry_pose[0]:.2f}, "
            f"{self.entry_pose[1]:.2f}), heading={math.degrees(yaw):.1f} deg"
        )
        return True

    # ------------------------------------------------------------------ #
    # fitting
    # ------------------------------------------------------------------ #
    def _try_fit(self) -> None:
        if self._cones is None or self._colors is None:
            self._warn_wait("no cone map yet")
            return
        if not self._ensure_entry_frame():
            self._warn_wait("waiting for start pose (TF track<-base_link)")
            return

        boundary = select_boundary_cones(self._cones, self._colors)
        car_pos = self.entry_pose
        fit, diag = fit_skidpad(
            boundary, self.entry_heading, car_pos, self.fit_params
        )
        self.fit_diag = diag
        if fit is None:
            reasons = diag.get("reasons") or ["unknown"]
            self.freeze_streak = 0
            self._last_centres = None
            self._warn_wait(
                f"boundary cones={len(boundary)}, reasons: " + "; ".join(reasons)
            )
            return

        warnings = diag.get("warnings") or []
        if warnings:
            self.get_logger().warn(
                "skidpad fit warnings: " + "; ".join(warnings),
                throttle_duration_sec=2.0,
            )

        if self._last_centres is None:
            drift = float("inf")
        else:
            drift = max(
                float(np.linalg.norm(fit.c_right - self._last_centres[0])),
                float(np.linalg.norm(fit.c_left - self._last_centres[1])),
            )
        self._last_centres = (fit.c_right.copy(), fit.c_left.copy())
        if drift <= self.freeze_drift:
            self.freeze_streak += 1
        else:
            self.freeze_streak = 1

        self.get_logger().info(
            f"skidpad fit OK: R={fit.R:.3f} m, centres={fit.centre_dist:.3f} m, "
            f"rms={fit.rms:.3f} m, n={fit.n_cones}, removed={fit.removed}, "
            f"freeze {self.freeze_streak}/{self.freeze_updates} (drift {drift:.3f} m)",
            throttle_duration_sec=1.0,
        )
        if self.freeze_streak >= self.freeze_updates:
            self._freeze(fit)

    def _freeze(self, fit) -> None:
        if self.entry_len_param > 0.0:
            entry_len = self.entry_len_param
        else:
            base = fit.entry_along if fit.entry_along is not None else 15.0
            entry_len = min(
                self.entry_len_max, max(self.entry_len_min, float(base) + 2.0)
            )
        try:
            path = generate_skidpad_path(
                fit.c_right,
                fit.c_left,
                fit.R,
                self.entry_heading,
                entry_len,
                self.exit_len,
                self.ds,
            )
        except ValueError as exc:
            self.get_logger().error(f"skidpad path generation failed: {exc}")
            self.freeze_streak = 0
            return

        errors = validate_skidpad_path(path, ds=self.ds)
        if errors:
            self.get_logger().error(
                "skidpad path self-check failed; not publishing: "
                + "; ".join(errors)
            )
            self.freeze_streak = 0
            return

        speeds, curvatures = compute_skidpad_speeds(
            path,
            speed_cap=self.speed_cap,
            params=self.speed_params,
            smoothing_window=self.curvature_smoothing,
            stop_len=self.stop_len,
        )
        self.path = path
        self.speeds = speeds
        self.curvatures = curvatures
        self.progress = SkidpadProgress(
            path,
            ProgressConfig(
                search_back=float(self.get_parameter("progress_search_back").value),
                search_ahead=float(self.get_parameter("progress_search_ahead").value),
                window_len=self.window_len,
                off_path_dist=float(self.get_parameter("off_path_dist").value),
                off_path_time=float(self.get_parameter("off_path_time").value),
            ),
        )
        pose = self._lookup_pose()
        if pose is not None:
            self.progress.initialize(float(pose[0][0]), float(pose[0][1]))
        self.state = self.progress.state if self.progress.initialized else STATE_WAITING
        self.frozen = True
        self._finished_logged = False
        self._fault_logged = False

        self._publish_full_path()
        self._publish_markers()

        self.get_logger().info(
            "skidpad template FROZEN: "
            f"R={fit.R:.3f} m, r_in={fit.r_in:.3f}, r_out={fit.r_out:.3f}, "
            f"centres=({fit.c_right[0]:.2f},{fit.c_right[1]:.2f})/"
            f"({fit.c_left[0]:.2f},{fit.c_left[1]:.2f}), "
            f"rms={fit.rms:.3f}, p90={fit.p90:.3f}, n={fit.n_cones}, "
            f"removed={fit.removed}, entry_len={entry_len:.1f}, "
            f"total={path.total_length:.1f} m"
        )

    def _warn_wait(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_wait_log >= 1.0:
            self._last_wait_log = now
            self.get_logger().warn(f"skidpad waiting for fit: {text}")

    # ------------------------------------------------------------------ #
    # periodic publishing
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        now = time.monotonic()
        if not self.frozen and self.state == STATE_WAITING:
            if (
                self._map_seq != self._last_fit_seq
                and now - self._last_fit_mono >= 1.0 / self.fit_rate
            ):
                self._last_fit_seq = self._map_seq
                self._last_fit_mono = now
                self._try_fit()

        if self.path is not None and not self.shutdown_latched:
            self._publish_driving()

        self._publish_state()

        if self.path is not None and now - self._last_marker_mono >= 1.0:
            self._last_marker_mono = now
            self._publish_markers()

    def _publish_driving(self) -> None:
        if self.progress is None or self.path is None:
            return
        if self.progress.state in (STATE_FINISHED, STATE_FAULT, STATE_SHUTDOWN):
            self.state = self.progress.state
            self._log_terminal()
            return

        pose = self._lookup_pose()
        if pose is None:
            # No TF: do not publish a new window. The tracker's 0.5 s
            # staleness timeout stops the car.
            return
        x, y = pose[0]
        upd = self.progress.update(float(x), float(y), time.monotonic())
        self.state = upd.state
        if upd.state in (STATE_FINISHED, STATE_FAULT):
            self._log_terminal()
            return
        if upd.window is None:
            return

        i0, i1 = upd.window
        msg = PathWithSpeeds()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self._path_poses(i0, i1, msg.header.stamp)
        msg.target_speeds = [float(v) for v in self.speeds[i0 : i1 + 1]]
        msg.curvatures = [float(v) for v in self.curvatures[i0 : i1 + 1]]
        self.path_pub.publish(msg)

    def _path_poses(self, i0: int, i1: int, stamp) -> List[PoseStamped]:
        assert self.path is not None
        poses: List[PoseStamped] = []
        for i in range(i0, i1 + 1):
            if i < len(self.path.points) - 1:
                d = self.path.points[i + 1] - self.path.points[i]
            else:
                d = self.path.points[i] - self.path.points[i - 1]
            yaw = math.atan2(float(d[1]), float(d[0]))
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = stamp
            pose.pose.position.x = float(self.path.points[i, 0])
            pose.pose.position.y = float(self.path.points[i, 1])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            poses.append(pose)
        return poses

    def _publish_full_path(self) -> None:
        if self.path is None:
            return
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self._path_poses(0, len(self.path.points) - 1, msg.header.stamp)
        self.full_path_pub.publish(msg)

    def _publish_state(self) -> None:
        if self.progress is not None:
            s_prog = self.progress.s_prog
            seg = self.path.segment_at(s_prog) if self.path is not None else "ENTRY"
            lap = SEGMENT_LAP.get(seg, 0)
            state = self.progress.state
        else:
            s_prog = 0.0
            seg = "ENTRY"
            lap = 0
            state = self.state

        diag = self.fit_diag or {}
        state_msg = {
            "state": state,
            "segment": seg,
            "lap": lap,
            "s_prog": round(float(s_prog), 3),
            "total_length": round(float(self.path.total_length), 3)
            if self.path is not None
            else None,
            "boundary_cones": int(diag.get("n_input", 0)),
            "fit_ok": bool(self.frozen),
            "fit_rms": diag.get("rms"),
            "fit_p90": diag.get("p90"),
            "fit_R": diag.get("R"),
            "fit_ring_width": diag.get("ring_width"),
            "fit_centre_dist": diag.get("centre_dist"),
            "fit_removed": diag.get("removed"),
            "fit_reasons": diag.get("reasons", []),
            "fit_warnings": diag.get("warnings", []),
        }
        msg = String()
        msg.data = json.dumps(state_msg)
        self.state_pub.publish(msg)

    def _log_terminal(self) -> None:
        if self.state == STATE_FINISHED and not self._finished_logged:
            self._finished_logged = True
            self.get_logger().info("skidpad FINISHED: full path completed")
        if self.state == STATE_FAULT and not self._fault_logged:
            self._fault_logged = True
            self.get_logger().error(
                "skidpad FAULT: car left the path (>2 m for >1 s); path publishing stopped"
            )

    # ------------------------------------------------------------------ #
    # markers
    # ------------------------------------------------------------------ #
    def _publish_markers(self) -> None:
        if self.path is None:
            return
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = stamp
        line.ns = "skidpad"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.12
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 0.2
        line.color.a = 1.0
        line.points = [
            _point(p[0], p[1], 0.02) for p in self.path.points
        ]
        arr.markers.append(line)

        for idx, (centre, color) in enumerate(
            (
                (self.path.c_right, (0.1, 0.4, 1.0)),
                (self.path.c_left, (1.0, 0.7, 0.1)),
                (self.path.o, (1.0, 1.0, 1.0)),
            )
        ):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = "skidpad_centres"
            m.id = idx + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.5 if idx < 2 else 0.35
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            m.pose.position = _point(centre[0], centre[1], 0.1)
            arr.markers.append(m)

        self.marker_pub.publish(arr)

    # ------------------------------------------------------------------ #
    # shutdown / reset
    # ------------------------------------------------------------------ #
    def _shutdown_cb(self, msg: Shutdown) -> None:
        if msg.emergency_shutdown or msg.finished_engage_ebs or msg.finished:
            if not self.shutdown_latched:
                self.get_logger().error("system shutdown received; skidpad path stopped")
            self.shutdown_latched = True
            self.state = STATE_SHUTDOWN
            if self.progress is not None:
                self.progress.shutdown()

    def _reset_cb(self, msg: Reset) -> None:
        if not msg.reset:
            return
        self.get_logger().warn("skidpad reset: clearing fit and progress")
        self.shutdown_latched = False
        self.frozen = False
        self.fit = None
        self.fit_diag = {"reasons": [], "warnings": []}
        self.freeze_streak = 0
        self._last_centres = None
        self.path = None
        self.speeds = None
        self.curvatures = None
        self.progress = None
        self.state = STATE_WAITING
        self.entry_pose = None
        self.entry_heading = None
        self._finished_logged = False
        self._fault_logged = False
        self._last_fit_seq = -1


def _point(x: float, y: float, z: float):
    from geometry_msgs.msg import Point

    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkidpadPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
