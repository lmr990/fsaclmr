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

from math import pi

import numpy as np
import scipy.interpolate as scipy_interpolate
import rclpy
from rclpy.node import Node
from transforms3d.euler import euler2quat
from typing import List, Tuple

from driverless_msgs.msg import PathLimits, PathWithSpeeds
from driverless_common.common import QOS_LATEST
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

from planners.speed_profile import SpeedProfileParams, compute_curvatures, compute_speed_profile


def approximate_b_spline_path(x: list, y: list, n_path_points: int, degree=3, s=0) -> Tuple[list, list]:
    """
    ADAPTED FROM: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/BSplinePath/bspline_path.py (same
    helper as in node_ft_planner.py) \n
    Approximate points with a B-Spline path
    """
    dx, dy = np.diff(x), np.diff(y)
    distances = np.cumsum([np.hypot(idx, idy) for idx, idy in zip(dx, dy)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]

    spl_i_x = scipy_interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = scipy_interpolate.UnivariateSpline(distances, y, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    return spl_i_x(sampled), spl_i_y(sampled)


def make_path_msg(points, frame) -> Path:
    """Same pose-building as node_ft_planner.make_path_msg (yaw from consecutive points)."""
    spline_len = len(points)
    poses: List = []
    for i in range(spline_len):
        if i < spline_len - 1:
            th_change = np.arctan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0])
        else:
            th_change = np.arctan2(points[0][1] - points[i][1], points[0][0] - points[i][0])
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

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
        self.path_with_speeds_pub = self.create_publisher(
            PathWithSpeeds, self.get_parameter("path_with_speeds_topic").value, QOS_LATEST
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1 / rate, self.publish_latest)

        self.get_logger().info("---UrinayPathBridge node initialised---")

    def velocity_callback(self, msg: Float32):
        self.current_velocity = msg.data

    def pathlimits_callback(self, msg: PathLimits):
        if len(msg.path) < 2:
            return
        self.centerline = np.array([[p.x, p.y] for p in msg.path])
        self.source_stamp = msg.header.stamp

    def publish_latest(self):
        if self.centerline is None:
            return

        if len(self.centerline) >= self.min_spline_points:
            n_path_points = self.spline_const * len(self.centerline)
            spline_x, spline_y = approximate_b_spline_path(
                self.centerline[:, 0].tolist(), self.centerline[:, 1].tolist(), n_path_points
            )
            midline = np.column_stack((spline_x, spline_y))
        else:
            midline = self.centerline

        self.path_pub.publish(make_path_msg(midline, self.map_frame))

        curvatures = compute_curvatures(midline, self.curvature_smoothing_window)
        params = SpeedProfileParams(**vars(self.sp_params))
        if self.use_velocity_seed:
            params.current_speed = max(0.0, self.current_velocity)
        speeds = compute_speed_profile(midline, curvatures, params)

        msg = PathWithSpeeds()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.source_stamp if self.source_stamp is not None else self.get_clock().now().to_msg()
        msg.poses = make_path_msg(midline, self.map_frame).poses
        msg.target_speeds = [float(v) for v in speeds]
        msg.curvatures = [float(k) for k in curvatures]
        self.path_with_speeds_pub.publish(msg)


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
