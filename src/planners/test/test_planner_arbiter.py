"""Unit tests for the shadow-mode planner arbiter helpers (no ROS spinning)."""

import os

import numpy as np
import pytest

from planners.node_planner_arbiter import (
    centerline_from_track_csv,
    compare_paths,
    corridor_score,
    path_ahead_window,
    track_csv_path,
)


def straight_path(length=30.0, step=0.5, y=0.0):
    x = np.arange(0.0, length + step / 2, step)
    return np.column_stack([x, np.full_like(x, y)])


class TestPathAheadWindow:
    def test_window_starts_at_point_nearest_the_car(self):
        path = straight_path()
        car = np.array([10.0, 0.0])
        win = path_ahead_window(path, car, horizon_m=5.0, step_m=0.5)
        assert win is not None
        # The window begins at the car's arc position, not at the path origin.
        assert np.isclose(win[0][0], 10.0)
        # Resampled at the requested step (arange excludes the horizon end).
        assert np.isclose(win[-1][0], 14.5)
        assert np.allclose(np.diff(win[:, 0]), 0.5)

    def test_window_truncated_at_path_end(self):
        path = straight_path(length=12.0)
        car = np.array([10.0, 0.0])
        win = path_ahead_window(path, car, horizon_m=20.0, step_m=0.5)
        assert win is not None
        assert np.isclose(win[-1][0], 11.5)

    def test_history_behind_car_is_excluded(self):
        path = straight_path()
        car = np.array([20.0, 0.0])
        win = path_ahead_window(path, car, horizon_m=30.0, step_m=0.5)
        assert win[0][0] >= 20.0

    def test_too_short_path_returns_none(self):
        path = np.array([[0.0, 0.0], [0.1, 0.0]])
        assert path_ahead_window(path, np.array([0.0, 0.0]), 5.0, 0.5) is None


class TestComparePaths:
    def test_identical_paths_deviate_zero(self):
        win = straight_path(length=10.0)
        mean, mx = compare_paths(win, win.copy())
        assert np.isclose(mean, 0.0)
        assert np.isclose(mx, 0.0)

    def test_parallel_offset_measured(self):
        active = straight_path(length=10.0, y=0.0)
        shadow = straight_path(length=10.0, y=0.8)
        mean, mx = compare_paths(active, shadow)
        assert np.isclose(mean, 0.8)
        assert np.isclose(mx, 0.8)

    def test_comparison_truncated_to_shorter_window(self):
        # Shadow path is offset but ends early: the distance beyond its
        # frontier would measure path length (growing to ~10 m), not
        # divergence, so truncation must keep max at the true offset.
        active = straight_path(length=20.0)
        shadow = straight_path(length=10.0, y=0.3)
        mean, mx = compare_paths(active, shadow)
        assert np.isclose(mean, 0.3)
        assert np.isclose(mx, 0.3)


class TestCorridorScore:
    def test_centre_of_corridor_scores_half_width(self):
        # Dense cones so the along-track sampling gap contributes < 0.05 m.
        xs = np.arange(0, 12, 0.5)
        cones = np.array([[x, 1.5] for x in xs] + [[x, -1.5] for x in xs])
        win = straight_path(length=10.0, y=0.0)  # centreline between the rows
        score = corridor_score(win, cones)
        assert np.isclose(score, 1.5, atol=0.05)


@pytest.fixture
def oval_csv(tmp_path):
    """A synthetic EUFS-style track CSV: two concentric boundary rings."""
    def ring(radius, n, phase=0.0):
        theta = phase + np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        return np.column_stack([radius * np.cos(theta), radius * np.sin(theta)])

    blue = ring(11.75, 40)
    yellow = ring(8.25, 30)  # different count and phase from blue
    rows = ["tag,x,y,direction,x_variance,y_variance,xy_covariance"]
    rows += [f"blue,{p[0]:.3f},{p[1]:.3f},0,0,0,0" for p in blue]
    rows += [f"yellow,{p[0]:.3f},{p[1]:.3f},0,0,0,0" for p in yellow]
    rows += [f"big_orange,{p[0]:.3f},{p[1]:.3f},0,0,0,0" for p in ring(10.0, 2)]
    rows.append("car_start,0.0,-10.0,0,0,0,0")
    path = tmp_path / "oval.csv"
    path.write_text("\n".join(rows))
    return str(path)


class TestCenterlineFromTrackCsv:
    def test_midpoints_lie_on_the_true_centreline(self, oval_csv):
        gt = centerline_from_track_csv(oval_csv, n_samples=100)
        assert gt is not None and len(gt) == 100
        radii = np.hypot(gt[:, 0], gt[:, 1])
        assert np.isclose(np.mean(radii), 10.0, atol=0.15)
        assert radii.max() - radii.min() < 0.5

    def test_missing_file_returns_none(self, tmp_path):
        assert centerline_from_track_csv(str(tmp_path / "nope.csv")) is None

    def test_too_few_boundary_cones_returns_none(self, tmp_path):
        path = tmp_path / "tiny.csv"
        path.write_text("tag,x,y\nblue,0,0\nblue,1,1\nyellow,2,2\n")
        assert centerline_from_track_csv(str(path)) is None

    def test_track_csv_path_resolution(self, oval_csv, tmp_path, monkeypatch):
        assert track_csv_path("") is None
        assert track_csv_path("does_not_exist_track") is None
        # Point the ament share index at a fake eufs_tracks share dir.
        fake_share = tmp_path / "eufs_tracks" / "share" / "eufs_tracks" / "csv"
        fake_share.mkdir(parents=True)
        (fake_share / "oval.csv").write_text("tag,x,y\n")
        monkeypatch.setattr(
            "planners.node_planner_arbiter.get_package_share_directory",
            lambda name: str(tmp_path / "eufs_tracks" / "share" / "eufs_tracks"),
        )
        assert track_csv_path("oval") == str(fake_share / "oval.csv")

    def test_no_cones_returns_nan(self):
        win = straight_path(length=5.0)
        assert np.isnan(corridor_score(win, None))
        assert np.isnan(corridor_score(win, np.empty((0, 2))))
