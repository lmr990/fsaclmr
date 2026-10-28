"""Bridge urinay's PathLimits output onto the existing planning contract.

Urinay publishes a centerline + track limits (driverless_msgs/PathLimits)
whenever the cone map updates. The downstream controllers expect:
  - /planning/midline_path      nav_msgs/Path
  - /planning/path_with_speeds  driverless_msgs/PathWithSpeeds
    (poses, target_speeds, curvatures, header.stamp = source detection stamp)

This node converts the latest PathLimits (partial or full) to that contract,
reusing the same three-pass speed profile as the ft_planner, and republishes
at a fixed rate so the path tracker's freshness timeout is satisfied even
when cone map updates are slow.
"""

import time
from math import pi

import numpy as np
import scipy.interpolate as scipy_interpolate
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from transforms3d.euler import euler2quat
from typing import List, Tuple

from driverless_msgs.msg import ConeWithIdArray, PathLimits, PathWithSpeeds
from driverless_common.common import QOS_LATEST
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

from planners.speed_profile import SpeedProfileParams, compute_curvatures, compute_speed_profile


def approximate_b_spline_path(
    x: list, y: list, n_path_points: int, degree=3, s=0, periodic=False
) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py (same
    helper as in node_ft_planner.py)\n
    Approximate points with a B-Spline path.

    With ``periodic=True`` the first/last (duplicate) sample is dropped and a
    periodic cubic spline is fitted, so a full loop has no seam discontinuity.
    The samples then exclude the duplicated endpoint.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if periodic and len(x) >= 4:
        # urinay's full path repeats the loop closure point; drop it and let
        # the periodic spline span the closing segment.
        x, y = x[:-1], y[:-1]

    dx, dy = np.diff(x), np.diff(y)
    segments = np.hypot(dx, dy)
    if periodic:
        segments = np.append(segments, np.hypot(x[0] - x[-1], y[0] - y[-1]))
    distances = np.concatenate(([0.0], np.cumsum(segments)))
    if distances[-1] <= 1e-9:
        return x, y
    distances /= distances[-1]

    if periodic:
        xs = np.append(x, x[0])
        ys = np.append(y, y[0])
        spl_i_x = scipy_interpolate.CubicSpline(distances, xs, bc_type="periodic")
        spl_i_y = scipy_interpolate.CubicSpline(distances, ys, bc_type="periodic")
        sampled = np.linspace(0.0, distances[-1], n_path_points, endpoint=False)
    else:
        spl_i_x = scipy_interpolate.UnivariateSpline(distances, x, k=degree, s=s)
        spl_i_y = scipy_interpolate.UnivariateSpline(distances, y, k=degree, s=s)
        sampled = np.linspace(0.0, distances[-1], n_path_points)
    return spl_i_x(sampled), spl_i_y(sampled)


def clamp_path_to_cone_corridor(
    points: np.ndarray,
    cone_pts: np.ndarray,
    max_dist: float,
    extrap_len: float,
    extrap_step: float,
) -> Tuple[np.ndarray, bool]:
    """Truncate the path where its tail leaves the cone corridor.

    The planner's frontier (the path end) is where sparse or cross-infield
    geometry can bend the centerline out of the real corridor. A point is
    corridor-valid when it lies within ``max_dist`` of the nearest mapped
    cone. Only the tail is acted on: an off-corridor blip in the middle with
    a healthy end is left alone (the tracker recovers from local noise), but
    a tail that leaves the corridor would steer the car off the track.

    The truncated end is simply dropped: the bridge republishes at a fixed
    rate, so the path grows again on the next centerline once cones ahead
    enter the map, while the speed profile brakes only as the car genuinely
    runs out of path. Set ``extrap_len`` > 0 to instead append a straight
    segment along the final heading (sampled every ``extrap_step``).

    Returns ``(points, truncated)``.
    """
    if len(points) < 2 or len(cone_pts) == 0:
        return points, False
    dists = cKDTree(cone_pts).query(points)[0]
    if dists[-1] <= max_dist:
        return points, False

    valid = np.nonzero(dists <= max_dist)[0]
    if valid.size == 0:
        return points[:1], True
    k = int(valid[-1])
    kept = points[: k + 1]
    if extrap_len <= 0.0:
        return kept, True

    # Straight extension along the mean of the last few headings.
    back = kept[max(0, len(kept) - 3)]
    heading = np.arctan2(kept[-1][1] - back[1], kept[-1][0] - back[0])
    direction = np.array([np.cos(heading), np.sin(heading)])
    n_extra = max(1, int(round(extrap_len / extrap_step)))
    steps = (np.arange(1, n_extra + 1) * extrap_step)[:, None]
    extension = kept[-1] + steps * direction
    return np.vstack((kept, extension)), True


def make_path_msg(points, frame, closed: bool = False) -> Path:
    """Pose-building with yaw from consecutive points.

    ``closed`` only affects the last pose: for a loop the heading points across
    the seam, for an open path it follows the previous segment instead of
    pointing back at the first point.
    """
    spline_len = len(points)
    poses: List = []
    for i in range(spline_len):
        if i < spline_len - 1:
            th_change = np.arctan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0])
        elif closed and spline_len > 1:
            th_change = np.arctan2(points[0][1] - points[i][1], points[0][0] - points[i][0])
        elif spline_len > 1:
            th_change = np.arctan2(points[i][1] - points[i - 1][1], points[i][0] - points[i - 1][0])
        else:
            th_change = 0.0
        if th_change > pi:
            th_change = th_change - 2 * pi
        elif th_change < -pi:
            th_change = th_change + 2 * pi

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = float(points[i][0])
        pose.pose.position.y = float(points[i][1])
        pose.pose.position.z = 0.0
        quat = euler2quat(0.0, 0.0, th_change)
        pose.pose.orientation.w = quat[0]
        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        poses.append(pose)

    path_msg = Path()
    path_msg.header.frame_id = frame
    path_msg.poses = poses
    return path_msg


class UrinayPathBridge(Node):
    spline_const = 10  # number of spline points per centerline point
    min_spline_points = 5  # below this the raw centerline is published as-is

    def __init__(self):
        super().__init__("urinay_path_bridge")

        self.map_frame = self.declare_parameter("map_frame", "track").value
        self.declare_parameter("partial_topic", "/AS/P/pathlimits/partial")
        self.declare_parameter("full_topic", "/AS/P/pathlimits/full")
        self.declare_parameter("path_topic", "/planning/midline_path")
        self.declare_parameter("path_with_speeds_topic", "/planning/path_with_speeds")
        self.declare_parameter("publish_rate", 10.0)
        # Stop republishing when urinay stops producing paths: the planner can
        # exit (loop closure, crash) and a stale path would keep driving the car.
        self.declare_parameter("path_timeout_sec", 1.0)
        # First/last centerline points closer than this [m] -> full loop.
        self.declare_parameter("closed_loop_gap_tol", 1.5)

        # Corridor guard: the frontier of the centerline can bend out of the
        # real corridor (sparse far cones, cross-infield triangulation on
        # small tracks — seen as the car being guided off track from lap 2
        # on). Truncate the path where its tail strays farther than
        # max_dist_to_cone from every mapped cone and drop the rest; the next
        # centerline reconnects as new cones enter the map. extrapolate_len
        # > 0 optionally appends a straight segment instead of dropping.
        self.declare_parameter("corridor.enabled", True)
        self.declare_parameter("corridor.max_dist_to_cone", 3.0)
        self.declare_parameter("corridor.extrapolate_len", 0.0)
        self.declare_parameter("corridor.extrapolate_step", 0.5)
        self.declare_parameter("cone_map_topic", "/urinay/cone_map")

        # three-pass speed profile parameters (same defaults as ft_planner.yaml)
        self.declare_parameter("speed_profile.speed_cap", 6.0)
        self.declare_parameter("speed_profile.max_lateral_acc", 5.0)
        self.declare_parameter("speed_profile.max_accel", 2.5)
        self.declare_parameter("speed_profile.max_brake", 4.0)
        self.declare_parameter("speed_profile.min_speed", 2.0)
        self.declare_parameter("speed_profile.curvature_lookahead_m", 5.0)
        self.declare_parameter("speed_profile.decel_to_stop_at_end", True)
        self.declare_parameter("curvature_smoothing_window", 3)
        self.declare_parameter("velocity_topic", "/vehicle/velocity")
        self.declare_parameter("use_velocity_seed", True)

        self.sp_params = SpeedProfileParams(
            speed_cap=self.get_parameter("speed_profile.speed_cap").value,
            max_lateral_acc=self.get_parameter("speed_profile.max_lateral_acc").value,
            max_accel=self.get_parameter("speed_profile.max_accel").value,
            max_brake=self.get_parameter("speed_profile.max_brake").value,
            min_speed=self.get_parameter("speed_profile.min_speed").value,
            curvature_lookahead_m=self.get_parameter("speed_profile.curvature_lookahead_m").value,
            decel_to_stop_at_end=self.get_parameter("speed_profile.decel_to_stop_at_end").value,
        )
        self.curvature_smoothing_window = self.get_parameter("curvature_smoothing_window").value
        self.use_velocity_seed = self.get_parameter("use_velocity_seed").value
        self.current_velocity = 0.0
        self.create_subscription(
            Float32,
            self.get_parameter("velocity_topic").value,
            self.velocity_callback,
            QOS_LATEST,
        )

        # latest centerline received from urinay (full overrides partial)
        self.centerline: np.ndarray = None
        self.source_stamp = None
        self.last_rx_mono = None
        # boundary cones carried by PathLimits (Point arrays)
        self.tracklimits_left = None
        self.tracklimits_right = None
        # Derived path, rebuilt only when a new centerline arrives. Speeds are
        # still recomputed every publish because they depend on vehicle speed.
        self.midline: np.ndarray = None
        self.poses: list = []
        self.curvatures: np.ndarray = None
        self.path_msg: Path = None
        self.closed = False

        partial_topic = self.get_parameter("partial_topic").value
        full_topic = self.get_parameter("full_topic").value
        # the full path is latched (transient local) on the urinay side
        full_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PathLimits, partial_topic, self.pathlimits_callback, QOS_LATEST)
        self.create_subscription(PathLimits, full_topic, self.pathlimits_callback, full_qos)

        self.corridor_enabled = bool(self.get_parameter("corridor.enabled").value)
        self.max_dist_to_cone = float(self.get_parameter("corridor.max_dist_to_cone").value)
        self.extrapolate_len = float(self.get_parameter("corridor.extrapolate_len").value)
        self.extrapolate_step = float(self.get_parameter("corridor.extrapolate_step").value)
        # Latest cumulative cone map (same data urinay triangulates); the
        # corridor guard measures against it. None = no map yet / cleared.
        self._cone_pts = None
        self.create_subscription(
            ConeWithIdArray,
            self.get_parameter("cone_map_topic").value,
            self.cone_map_callback,
            QOS_LATEST,
        )

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
        self.path_with_speeds_pub = self.create_publisher(
            PathWithSpeeds, self.get_parameter("path_with_speeds_topic").value, QOS_LATEST
        )
        # Track limits from urinay's PathLimits republished as nav_msgs/Path
        # under the same names the ft planner uses — consumers (e.g. the
        # path overlay corridor band) then work with either planner.
        self.declare_parameter("blue_bounds_topic", "/planning/blue_bounds")
        self.declare_parameter("yellow_bounds_topic", "/planning/yellow_bounds")
        self.blue_bounds_pub = self.create_publisher(
            Path, self.get_parameter("blue_bounds_topic").value, 1
        )
        self.yellow_bounds_pub = self.create_publisher(
            Path, self.get_parameter("yellow_bounds_topic").value, 1
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1 / rate, self.publish_latest)

        self.get_logger().info("---UrinayPathBridge node initialised---")

    def velocity_callback(self, msg: Float32):
        self.current_velocity = msg.data

    def cone_map_callback(self, msg: ConeWithIdArray):
        if msg.cones:
            self._cone_pts = np.array([[c.position.x, c.position.y] for c in msg.cones])
        else:
            self._cone_pts = None

    def pathlimits_callback(self, msg: PathLimits):
        if len(msg.path) < 2:
            return
        self.centerline = np.array([[p.x, p.y] for p in msg.path])
        self.source_stamp = msg.header.stamp
        self.last_rx_mono = time.monotonic()
        self.tracklimits_left = list(msg.tracklimits_left)
        self.tracklimits_right = list(msg.tracklimits_right)
        self._rebuild_path()

    def _rebuild_path(self):
        """Spline + poses + curvatures: once per new centerline, not per publish."""
        gap_tol = float(self.get_parameter("closed_loop_gap_tol").value)
        self.closed = bool(
            len(self.centerline) >= 4
            and np.linalg.norm(self.centerline[0] - self.centerline[-1]) <= gap_tol
        )
        if len(self.centerline) >= self.min_spline_points:
            n_path_points = self.spline_const * len(self.centerline)
            spline_x, spline_y = approximate_b_spline_path(
                self.centerline[:, 0].tolist(),
                self.centerline[:, 1].tolist(),
                n_path_points,
                periodic=self.closed,
            )
            self.midline = np.column_stack((spline_x, spline_y))
        else:
            self.midline = self.centerline
            self.closed = False

        if self.corridor_enabled and self._cone_pts is not None:
            self.midline, truncated = clamp_path_to_cone_corridor(
                self.midline,
                self._cone_pts,
                self.max_dist_to_cone,
                self.extrapolate_len,
                self.extrapolate_step,
            )
            if truncated:
                self.closed = False
                self.get_logger().warn(
                    "Path tail left the cone corridor; truncated and extended straight",
                    throttle_duration_sec=5.0,
                )

        path_msg = make_path_msg(self.midline, self.map_frame, closed=self.closed)
        path_msg.header.stamp = (
            self.source_stamp if self.source_stamp is not None
            else self.get_clock().now().to_msg()
        )
        self.path_msg = path_msg
        self.poses = path_msg.poses
        self.curvatures = compute_curvatures(
            self.midline, self.curvature_smoothing_window
        )

    def publish_latest(self):
        if self.path_msg is None:
            return
        if self.last_rx_mono is not None and (
            time.monotonic() - self.last_rx_mono > float(self.get_parameter("path_timeout_sec").value)
        ):
            # Upstream went silent: stop publishing so the tracker's own
            # staleness check brings the car to a safe stop.
            return

        self.path_pub.publish(self.path_msg)
        self._publish_tracklimits()

        params = SpeedProfileParams(**vars(self.sp_params))
        if self.use_velocity_seed:
            params.current_speed = max(0.0, self.current_velocity)
        speeds = compute_speed_profile(self.midline, self.curvatures, params)

        msg = PathWithSpeeds()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.path_msg.header.stamp
        msg.poses = self.poses
        msg.target_speeds = [float(v) for v in speeds]
        msg.curvatures = [float(k) for k in self.curvatures]
        self.path_with_speeds_pub.publish(msg)

    def _publish_tracklimits(self):
        """Republish urinay's boundary cones as blue/yellow bound Paths."""
        for pub, limits in (
            (self.blue_bounds_pub, self.tracklimits_left),
            (self.yellow_bounds_pub, self.tracklimits_right),
        ):
            if limits is None or len(limits) < 2:
                continue
            points = [[p.x, p.y] for p in limits]
            bound = make_path_msg(points, self.map_frame)
            bound.header.stamp = self.path_msg.header.stamp
            pub.publish(bound)


def main(args=None):
    rclpy.init(args=args)
    node = UrinayPathBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
