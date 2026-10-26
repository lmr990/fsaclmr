"""Unit tests for the three-pass speed profile (pure numpy, no ROS)."""

import numpy as np
import pytest

from planners.speed_profile import SpeedProfileParams, compute_curvatures, compute_speed_profile


def make_straight_path(n=50, spacing=1.0):
    x = np.arange(n) * spacing
    return np.column_stack([x, np.zeros(n)])


def make_circle_path(radius=10.0, n=100):
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])


class TestComputeCurvatures:
    def test_straight_line_is_zero(self):
        k = compute_curvatures(make_straight_path())
        assert np.allclose(k, 0.0, atol=1e-9)

    def test_circle_radius_recovered(self):
        radius = 10.0
        k = compute_curvatures(make_circle_path(radius), smoothing_window=0)
        # closed circle: every interior point should be ~1/R
        assert np.allclose(np.abs(k), 1.0 / radius, rtol=0.05)

    def test_short_path_returns_zeros(self):
        assert np.allclose(compute_curvatures(np.zeros((2, 2))), 0.0)
        assert np.allclose(compute_curvatures(np.zeros((1, 2))), 0.0)

    def test_sign_left_positive(self):
        # left turn (counter-clockwise) should give positive curvature (FLU frame)
        path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
        k = compute_curvatures(path, smoothing_window=0)
        assert k[1] > 0.0


class TestComputeSpeedProfile:
    def test_straight_line_capped(self):
        path = make_straight_path()
        k = np.zeros(len(path))
        p = SpeedProfileParams(speed_cap=6.0, current_speed=6.0, decel_to_stop_at_end=False)
        v = compute_speed_profile(path, k, p)
        assert v[0] == pytest.approx(6.0)
        assert v.max() <= 6.0 + 1e-9

    def test_end_of_path_decelerates(self):
        path = make_straight_path()
        k = np.zeros(len(path))
        p = SpeedProfileParams(speed_cap=6.0, current_speed=6.0, min_speed=2.0)
        v = compute_speed_profile(path, k, p)
        assert v[-1] <= 2.0 + 1e-9
        # braking must be feasible: no point above backward-reachable limit
        ds = np.linalg.norm(np.diff(path, axis=0), axis=1)
        for i in range(len(v) - 1, 0, -1):
            reachable = np.sqrt(v[i] ** 2 + 2.0 * p.max_brake * ds[i - 1])
            assert v[i - 1] <= reachable + 1e-9

    def test_corner_respects_lateral_limit(self):
        radius = 9.0  # v_lat = sqrt(5.0 / (1/9)) = sqrt(45) ~ 6.7 -> capped by 6
        p_small = SpeedProfileParams(speed_cap=20.0, max_lateral_acc=5.0, decel_to_stop_at_end=False)
        path = make_circle_path(radius)
        k = compute_curvatures(path, smoothing_window=0)
        v = compute_speed_profile(path, k, p_small)
        expected = np.sqrt(5.0 * radius)
        assert v.max() <= expected + 0.5  # smoothing keeps it near, not exactly at, the limit
        assert v.max() > expected * 0.8

    def test_forward_acceleration_respected(self):
        path = make_straight_path(n=100)
        k = np.zeros(len(path))
        p = SpeedProfileParams(
            speed_cap=6.0, max_accel=2.0, current_speed=0.0, decel_to_stop_at_end=False
        )
        v = compute_speed_profile(path, k, p)
        ds = 1.0
        for i in range(1, len(v)):
            reachable = np.sqrt(v[i - 1] ** 2 + 2.0 * 2.0 * ds) + 1e-9
            assert v[i] <= reachable

    def test_lookahead_brakes_before_corner(self):
        # straight 30m then a tight corner: speed must already be dropping
        # a few metres BEFORE the corner entry
        straight = np.column_stack([np.arange(30) * 1.0, np.zeros(30)])
        theta = np.linspace(0.0, np.pi, 30)
        corner = np.column_stack([30.0 + 5.0 * np.sin(theta), 5.0 - 5.0 * np.cos(theta)])
        path = np.vstack([straight, corner])
        k = compute_curvatures(path, smoothing_window=0)
        p = SpeedProfileParams(
            speed_cap=10.0, current_speed=10.0, max_brake=4.0, decel_to_stop_at_end=False
        )
        v = compute_speed_profile(path, k, p)
        entry_idx = 29  # last straight point
        pre_corner = v[entry_idx - 5]
        assert pre_corner < 10.0 - 0.5  # already slower than cap 5m before the corner

    def test_empty_path(self):
        v = compute_speed_profile(np.zeros((0, 2)), np.zeros(0), SpeedProfileParams())
        assert len(v) == 0

    def test_nan_curvature_tolerated(self):
        path = make_straight_path(10)
        k = np.full(10, np.nan)
        v = compute_speed_profile(path, k, SpeedProfileParams(speed_cap=3.0))
        assert np.all(np.isfinite(v))
