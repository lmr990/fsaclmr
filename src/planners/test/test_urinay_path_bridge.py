"""Unit tests for the urinay path bridge helpers (no ROS spinning)."""

import numpy as np

from planners.node_urinay_path_bridge import (
    approximate_b_spline_path,
    clamp_path_to_cone_corridor,
    make_path_msg,
)


def circle_centerline(radius=10.0, n=60):
    """Closed loop as urinay publishes it: the first point repeats at the end."""
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    points = np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])
    return np.vstack([points, points[0]])


class TestPeriodicSpline:
    def test_periodic_spline_has_no_seam_gap(self):
        points = circle_centerline()
        sx, sy = approximate_b_spline_path(
            points[:, 0].tolist(), points[:, 1].tolist(), 10 * len(points), periodic=True
        )
        radius = np.hypot(sx, sy)
        assert np.allclose(radius, 10.0, rtol=0.02)
        # Sampling excludes the duplicated endpoint, so the seam gap must be
        # about one sample spacing rather than a chord across the loop.
        seam = np.hypot(sx[0] - sx[-1], sy[0] - sy[-1])
        expected_spacing = 2.0 * np.pi * 10.0 / len(sx)
        assert seam < 3.0 * expected_spacing

    def test_open_path_keeps_endpoints(self):
        x = np.linspace(0.0, 10.0, 11).tolist()
        y = (np.zeros(11)).tolist()
        sx, sy = approximate_b_spline_path(x, y, 101)
        assert np.isclose(sx[0], 0.0)
        assert np.isclose(sx[-1], 10.0)


class TestMakePathMsg:
    def test_closed_path_heading_points_across_the_seam(self):
        points = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        msg = make_path_msg(points, "track", closed=True)
        # Last pose is (0,-1); the seam direction continues towards (1,0),
        # i.e. 45 deg, rather than pointing back towards (-1,0) (180 deg).
        q = msg.poses[-1].pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        assert np.isclose(yaw, np.pi / 4.0, atol=1e-6)

    def test_open_path_last_heading_follows_previous_segment(self):
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        msg = make_path_msg(points, "track", closed=False)
        q = msg.poses[-1].pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        assert abs(yaw) < 1e-6


def corridor_cones():
    """Two cone rows 3 m apart along +x, like a straight section of track."""
    left = [(x, 1.5) for x in np.arange(0.0, 40.0, 4.0)]
    right = [(x, -1.5) for x in np.arange(0.0, 40.0, 4.0)]
    return np.array(left + right)


class TestClampPathToConeCorridor:
    MAX_DIST = 3.0
    EXTRAP_LEN = 10.0
    EXTRAP_STEP = 0.5

    def test_healthy_path_is_untouched(self):
        x = np.arange(0.0, 30.0, 0.5)
        path = np.column_stack([x, np.zeros_like(x)])
        out, truncated = clamp_path_to_cone_corridor(
            path, corridor_cones(), self.MAX_DIST, self.EXTRAP_LEN, self.EXTRAP_STEP
        )
        assert not truncated
        assert np.allclose(out, path)

    def test_mid_path_blip_with_healthy_end_is_kept(self):
        # One waypoint pushed far outside the corridor, the tail back inside.
        x = np.arange(0.0, 30.0, 0.5)
        path = np.column_stack([x, np.zeros_like(x)])
        path[20] = (10.0, 20.0)
        out, truncated = clamp_path_to_cone_corridor(
            path, corridor_cones(), self.MAX_DIST, self.EXTRAP_LEN, self.EXTRAP_STEP
        )
        assert not truncated
        assert np.allclose(out, path)

    def test_off_corridor_tail_is_truncated_and_extended_straight(self):
        # Follow the corridor, then veer away like the frontier swing does.
        x = np.arange(0.0, 20.0, 0.5)
        y = np.concatenate([np.zeros(34), np.linspace(0.0, -30.0, 6)])
        path = np.column_stack([x, y])
        out, truncated = clamp_path_to_cone_corridor(
            path, corridor_cones(), self.MAX_DIST, self.EXTRAP_LEN, self.EXTRAP_STEP
        )
        assert truncated
        # Everything up to the last in-corridor point (index 34) is unchanged.
        assert np.allclose(out[:35], path[:35])
        # The extension continues along the final (+x) heading, evenly spaced.
        extension = out[35:]
        assert np.isclose(extension[0][0], path[34][0] + self.EXTRAP_STEP)
        assert np.allclose(np.diff(extension[:, 0]), self.EXTRAP_STEP)
        assert np.allclose(extension[:, 1], 0.0)
        # Total extension length matches the request.
        assert np.isclose(extension[-1][0] - extension[0][0] + self.EXTRAP_STEP,
                          self.EXTRAP_LEN)

    def test_dropped_tail_without_extrapolation(self):
        # extrapolate_len = 0: the off-corridor tail is simply dropped and
        # the next republished centerline is expected to reconnect.
        x = np.arange(0.0, 20.0, 0.5)
        y = np.concatenate([np.zeros(34), np.linspace(0.0, -30.0, 6)])
        path = np.column_stack([x, y])
        out, truncated = clamp_path_to_cone_corridor(path, corridor_cones(), self.MAX_DIST, 0.0, 0.5)
        assert truncated
        assert np.allclose(out, path[:35])

    def test_fully_off_corridor_path_collapses(self):
        path = np.column_stack([np.arange(10.0), np.full(10, 50.0)])
        out, truncated = clamp_path_to_cone_corridor(
            path, corridor_cones(), self.MAX_DIST, self.EXTRAP_LEN, self.EXTRAP_STEP
        )
        assert truncated
        assert len(out) == 1

    def test_empty_cone_map_disables_the_guard(self):
        path = np.column_stack([np.arange(10.0), np.zeros(10)])
        out, truncated = clamp_path_to_cone_corridor(
            path, np.empty((0, 2)), self.MAX_DIST, self.EXTRAP_LEN, self.EXTRAP_STEP
        )
        assert not truncated
        assert np.allclose(out, path)


class TestTracklimitsRepublish:
    """urinay's boundary cones must reach the ft-style bounds topics."""

    def _make_node_with_stub_pubs(self):
        import rclpy

        from planners.node_urinay_path_bridge import UrinayPathBridge

        if not rclpy.ok():
            rclpy.init()
        node = UrinayPathBridge()
        captured = {}
        for name in ("blue_bounds_pub", "yellow_bounds_pub"):
            captured[name] = []
            stub = type("Stub", (), {"publish": staticmethod(
                lambda msg, _name=name: captured[_name].append(msg))})()
            setattr(node, name, stub)
        yield node, captured
        node.destroy_node()
        rclpy.shutdown()

    def test_limits_become_bound_paths(self):
        from geometry_msgs.msg import Point
        from driverless_msgs.msg import PathLimits

        node, captured = next(self._make_node_with_stub_pubs())
        msg = PathLimits()
        msg.header.frame_id = "track"
        for x in (1.0, 2.0, 3.0):
            p = Point(); p.x, p.y = x, 1.5
            msg.tracklimits_left.append(p)
        for x in (1.0, 2.0, 3.0):
            p = Point(); p.x, p.y = x, -1.5
            msg.tracklimits_right.append(p)
        for x in (1.0, 2.0, 3.0):
            p = Point(); p.x, p.y = x, 0.0
            msg.path.append(p)

        node.pathlimits_callback(msg)          # stores limits, rebuilds path
        node.publish_latest()                  # 10 Hz tick publishes bounds

        assert len(captured["blue_bounds_pub"]) >= 1
        assert len(captured["yellow_bounds_pub"]) >= 1
        blue = captured["blue_bounds_pub"][-1]
        assert blue.header.frame_id == "track"
        assert len(blue.poses) == 3
        assert np.isclose(blue.poses[0].pose.position.y, 1.5)
