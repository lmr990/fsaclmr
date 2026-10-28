"""Shadow-mode planner arbiter.

Runs beside the active planner chain and records how a second (shadow)
planner would have driven, without ever publishing to the control topics:
the car's behaviour is exactly what it would be without this node.

Every tick it compares the active and shadow paths over the arc-length
window ahead of the car (lateral deviation via nearest-point distance) and
scores each path's corridor validity (mean distance to the nearest mapped
cone). Findings go to the log and optionally to a CSV file:

    t, dev_mean, dev_max, corridor_active, corridor_shadow, verdict

Divergence events (dev_max above threshold) are the interesting signal:
the two planners fail in orthogonal ways (urinay: geometric traps, ft:
colour misclassification), so persistent disagreement localizes whether a
perception drop or a planning trap is active.
"""

import csv
import os
import pathlib
import time
from datetime import datetime
from typing import Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformException, TransformListener

from driverless_msgs.msg import ConeWithIdArray
from driverless_common.common import QOS_LATEST
from nav_msgs.msg import Path


def path_ahead_window(
    path: np.ndarray, car_pos: np.ndarray, horizon_m: float, step_m: float
) -> Optional[np.ndarray]:
    """Arc-length resample of the path ahead of the car.

    ``path`` is an (N, 2) array ordered as published (history first, frontier
    last). The window starts at the point nearest to the car and extends
    ``horizon_m`` metres, sampled every ``step_m``. Returns None when the
    path is too short to resample.
    """
    if len(path) < 2:
        return None
    idx = int(np.argmin(np.linalg.norm(path - car_pos, axis=1)))
    ahead = path[idx:]
    if len(ahead) < 2:
        return None
    seg = np.linalg.norm(np.diff(ahead, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(seg)))
    if arc[-1] < step_m:
        return None
    targets = np.arange(0.0, min(arc[-1], horizon_m), step_m)
    return np.column_stack(
        (np.interp(targets, arc, ahead[:, 0]), np.interp(targets, arc, ahead[:, 1]))
    )


def compare_paths(
    active_win: np.ndarray, shadow_win: np.ndarray
) -> Tuple[float, float]:
    """Mean and max deviation from active to shadow over the window.

    Both windows are already resampled to the same step, so the deviation of
    each active sample is its distance to the nearest shadow sample. The
    comparison is truncated to the shorter window: beyond a planner's frontier
    the distance to its last point would measure path length, not divergence.
    """
    n = min(len(active_win), len(shadow_win))
    if n == 0:
        return float("inf"), float("inf")
    dists = cKDTree(shadow_win[:n]).query(active_win[:n])[0]
    return float(np.mean(dists)), float(np.max(dists))


def corridor_score(win: np.ndarray, cone_pts: Optional[np.ndarray]) -> float:
    """Mean distance to the nearest mapped cone over the window.

    Lower is more corridor-consistent. Returns NaN when no cone map has been
    received yet, so callers can skip the verdict rather than misrank.
    """
    if cone_pts is None or len(cone_pts) == 0 or len(win) == 0:
        return float("nan")
    return float(np.mean(cKDTree(cone_pts).query(win)[0]))


def _sorted_by_angle(pts: np.ndarray) -> np.ndarray:
    """Order boundary points anticlockwise around their centroid."""
    centroid = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - centroid[1], pts[:, 0] - centroid[0])
    return pts[np.argsort(angles)]


def _resample_periodic(pts: np.ndarray, n: int) -> Optional[np.ndarray]:
    """Resample a point ring to n equally spaced samples by arc length."""
    closed = np.vstack([pts, pts[0]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(seg)))
    if arc[-1] <= 1e-9:
        return None
    targets = np.linspace(0.0, arc[-1], n, endpoint=False)
    return np.column_stack(
        (np.interp(targets, arc, closed[:, 0]), np.interp(targets, arc, closed[:, 1]))
    )


def centerline_from_track_csv(csv_path: str, n_samples: int = 200) -> Optional[np.ndarray]:
    """Approximate true centreline from an EUFS track CSV (map frame).

    Pairs the blue and yellow boundaries: each is angle-sorted around its own
    centroid, resampled to n_samples by arc length, phase-aligned by trying
    every cyclic shift (so the two chains' starting points need not
    correspond), then averaged. Works well for convex loops; on non-convex
    tracks (hairpins) the pairing degrades, so callers should treat the
    result as a reference and sanity-check the reported track width.
    """
    blue: list = []
    yellow: list = []
    try:
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                tag = (row.get("tag") or "").strip().lower()
                if tag not in ("blue", "yellow"):
                    continue
                try:
                    point = (float(row["x"]), float(row["y"]))
                except (KeyError, TypeError, ValueError):
                    continue
                (blue if tag == "blue" else yellow).append(point)
    except OSError:
        return None
    if len(blue) < 3 or len(yellow) < 3:
        return None

    resampled_blue = _resample_periodic(_sorted_by_angle(np.array(blue)), n_samples)
    resampled_yellow = _resample_periodic(_sorted_by_angle(np.array(yellow)), n_samples)
    if resampled_blue is None or resampled_yellow is None:
        return None

    # Phase-align the two rings before averaging (cheapest correct pairing).
    shifts = np.arange(n_samples)
    best_shift, best_cost = 0, np.inf
    for shift in shifts:
        cost = float(np.sum((resampled_blue - np.roll(resampled_yellow, shift)) ** 2))
        if cost < best_cost:
            best_shift, best_cost = shift, cost
    return (resampled_blue + np.roll(resampled_yellow, best_shift)) / 2.0


def track_csv_path(track_name: str) -> Optional[str]:
    """Locate <eufs_tracks share>/csv/<track_name>.csv, None if absent."""
    if not track_name:
        return None
    try:
        candidate = os.path.join(
            get_package_share_directory("eufs_tracks"), "csv", f"{track_name}.csv"
        )
    except Exception:
        return None
    return candidate if os.path.isfile(candidate) else None


def _workspace_logs_dir() -> Optional[str]:
    """<workspace>/logs/arbiter for the workspace owning this package.

    Uses pathlib.Path spelled out because ``nav_msgs.msg.Path`` shadows the
    bare ``Path`` name in this module.
    """
    for parent in pathlib.Path(__file__).resolve().parents:
        if (parent / "install").is_dir() and (parent / "src").is_dir():
            return str(parent / "logs" / "arbiter")
    return None


class PlannerArbiter(Node):
    def __init__(self):
        super().__init__("planner_arbiter")

        self.declare_parameter("map_frame", "track")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("active_topic", "/planning/midline_path")
        self.declare_parameter("shadow_topic", "/shadow/ft/midline_path")
        self.declare_parameter("active_name", "active")
        self.declare_parameter("shadow_name", "shadow")
        self.declare_parameter("cone_map_topic", "/urinay/cone_map")
        self.declare_parameter("compare_horizon_m", 20.0)
        self.declare_parameter("compare_step_m", 0.5)
        # dev_max below this: the planners agree, log the periodic line only.
        self.declare_parameter("agree_max_dev_m", 1.0)
        # dev_max above this: log an immediate divergence event.
        self.declare_parameter("divergence_dev_m", 1.5)
        self.declare_parameter("log_period_sec", 2.0)
        # Ground-truth reference: score both paths against the centreline
        # computed from the EUFS track CSV (empty = corridor scoring only).
        self.declare_parameter("track_name", "")
        # CSV stats file. Empty path = <workspace>/logs/arbiter/<names>_<ts>.csv.
        self.declare_parameter("stats_csv_path", "")
        self.declare_parameter("stats_csv_dir", "")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.active_name = self.get_parameter("active_name").value
        self.shadow_name = self.get_parameter("shadow_name").value
        self.horizon = float(self.get_parameter("compare_horizon_m").value)
        self.step = float(self.get_parameter("compare_step_m").value)
        self.agree_max_dev = float(self.get_parameter("agree_max_dev_m").value)
        self.divergence_dev = float(self.get_parameter("divergence_dev_m").value)
        self.log_period = float(self.get_parameter("log_period_sec").value)

        # Ground-truth centreline (map frame) from the track CSV.
        self.gt_centerline: Optional[np.ndarray] = None
        track_name = self.get_parameter("track_name").value
        gt_path = track_csv_path(track_name)
        if gt_path:
            self.gt_centerline = centerline_from_track_csv(gt_path)
            if self.gt_centerline is not None:
                self.get_logger().info(
                    f"GT centreline from {gt_path}: {len(self.gt_centerline)} points"
                )
            else:
                self.get_logger().warn(f"Could not build GT centreline from {gt_path}")
        elif track_name:
            self.get_logger().warn(f"No track CSV for '{track_name}'; GT scoring disabled")

        csv_path = self.get_parameter("stats_csv_path").value
        if not csv_path:
            csv_dir = self.get_parameter("stats_csv_dir").value or _workspace_logs_dir()
            if csv_dir:
                try:
                    os.makedirs(csv_dir, exist_ok=True)
                    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    csv_path = os.path.join(
                        csv_dir, f"arbiter_{self.active_name}_vs_{self.shadow_name}_{stamp}.csv"
                    )
                except OSError:
                    csv_path = ""

        self.active_path: Optional[np.ndarray] = None
        self.shadow_path: Optional[np.ndarray] = None
        self.cone_pts: Optional[np.ndarray] = None
        self.divergence_count = 0

        self.create_subscription(
            Path, self.get_parameter("active_topic").value, self.active_callback, QOS_LATEST
        )
        self.create_subscription(
            Path, self.get_parameter("shadow_topic").value, self.shadow_callback, QOS_LATEST
        )
        self.create_subscription(
            ConeWithIdArray, self.get_parameter("cone_map_topic").value, self.cone_callback, QOS_LATEST
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.evaluate)

        self._csv_file = None
        self._csv_writer = None
        if csv_path:
            try:
                self._csv_file = open(csv_path, "a", newline="")
                self._csv_writer = csv.writer(self._csv_file)
                self._csv_writer.writerow(
                    ["t", "dev_mean", "dev_max", f"corridor_{self.active_name}",
                     f"corridor_{self.shadow_name}", f"gt_{self.active_name}",
                     f"gt_{self.shadow_name}", "verdict"]
                )
                self.get_logger().info(f"Writing arbiter stats to {csv_path}")
            except OSError as exc:
                self.get_logger().warn(f"Cannot open {csv_path}: {exc}")
                self._csv_file = None

        self.get_logger().info(
            f"---PlannerArbiter initialised (shadow only; publishes nothing): "
            f"{self.active_name}={self.get_parameter('active_topic').value} vs "
            f"{self.shadow_name}={self.get_parameter('shadow_topic').value}---"
        )

    def active_callback(self, msg: Path):
        self.active_path = self._path_to_array(msg)

    def shadow_callback(self, msg: Path):
        self.shadow_path = self._path_to_array(msg)

    def cone_callback(self, msg: ConeWithIdArray):
        self.cone_pts = (
            np.array([[c.position.x, c.position.y] for c in msg.cones]) if msg.cones else None
        )

    @staticmethod
    def _path_to_array(msg: Path) -> Optional[np.ndarray]:
        if len(msg.poses) < 2:
            return None
        return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def _car_pose(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y])

    def evaluate(self):
        if self.active_path is None or self.shadow_path is None:
            return
        car_pos = self._car_pose()
        if car_pos is None:
            return

        active_win = path_ahead_window(self.active_path, car_pos, self.horizon, self.step)
        shadow_win = path_ahead_window(self.shadow_path, car_pos, self.horizon, self.step)
        if active_win is None or shadow_win is None:
            return

        dev_mean, dev_max = compare_paths(active_win, shadow_win)
        corr_active = corridor_score(active_win, self.cone_pts)
        corr_shadow = corridor_score(shadow_win, self.cone_pts)
        gt_active = corridor_score(active_win, self.gt_centerline)
        gt_shadow = corridor_score(shadow_win, self.gt_centerline)

        if dev_max <= self.agree_max_dev:
            verdict = "agree"
            self._in_divergence = False
        else:
            verdict = "diverge"
            if not getattr(self, "_in_divergence", False):
                self.divergence_count += 1
                self._in_divergence = True
                if not (np.isnan(gt_active) or np.isnan(gt_shadow)):
                    better = self.active_name if gt_active <= gt_shadow else self.shadow_name
                    basis = "gt"
                elif not (np.isnan(corr_active) or np.isnan(corr_shadow)):
                    better = self.active_name if corr_active <= corr_shadow else self.shadow_name
                    basis = "corridor"
                else:
                    better, basis = "n/a", "none"
                self.get_logger().warn(
                    f"DIVERGENCE #{self.divergence_count}: dev_max={dev_max:.2f}m "
                    f"corridor {self.active_name}={corr_active:.2f}m "
                    f"{self.shadow_name}={corr_shadow:.2f}m | "
                    f"gt {self.active_name}={gt_active:.2f}m "
                    f"{self.shadow_name}={gt_shadow:.2f}m | better={better} ({basis})"
                )

        now_mono = time.monotonic()
        if now_mono - getattr(self, "_last_log", 0.0) < self.log_period:
            return
        self._last_log = now_mono
        self.get_logger().info(
            f"arbiter: dev_mean={dev_mean:.2f}m dev_max={dev_max:.2f}m | "
            f"corridor {self.active_name}={corr_active:.2f}m "
            f"{self.shadow_name}={corr_shadow:.2f}m | "
            f"gt {self.active_name}={gt_active:.2f}m {self.shadow_name}={gt_shadow:.2f}m | "
            f"{verdict} (events={self.divergence_count})"
        )
        if self._csv_writer is not None:
            self._csv_writer.writerow(
                [f"{now_mono:.1f}", f"{dev_mean:.3f}", f"{dev_max:.3f}",
                 f"{corr_active:.3f}", f"{corr_shadow:.3f}",
                 f"{gt_active:.3f}", f"{gt_shadow:.3f}", verdict]
            )
            self._csv_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._csv_file is not None:
            node._csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
