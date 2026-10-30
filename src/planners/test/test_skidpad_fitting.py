"""Unit tests for the skidpad template fitting (synthetic cones, no ROS)."""

import math

import numpy as np
import pytest

from planners.skidpad_fitting import FitParams, fit_skidpad, select_boundary_cones

R = 9.125
R_IN = 7.625
R_OUT = 10.625


def _template(entry_deg=90.0, n_per_row=16, noise=0.05, seed=0, drop=0.0, ghosts=0):
    rng = np.random.default_rng(seed)
    e = np.array(
        [math.cos(math.radians(entry_deg)), math.sin(math.radians(entry_deg))]
    )
    o = np.zeros(2)
    right_dir = np.array([e[1], -e[0]])
    c_right = o + R * right_dir
    c_left = o - R * right_dir
    points = []
    phase = rng.uniform(0.0, 2.0 * math.pi)
    for centre in (c_right, c_left):
        for radius in (R_IN, R_OUT):
            angles = np.linspace(0.0, 2.0 * math.pi, n_per_row, endpoint=False) + phase
            points.append(
                centre[None, :]
                + radius * np.column_stack((np.cos(angles), np.sin(angles)))
            )
    cones = np.vstack(points)
    cones += rng.normal(0.0, noise, size=cones.shape)
    if drop > 0.0:
        keep = rng.random(len(cones)) >= drop
        cones = cones[keep]
    if ghosts > 0:
        # Uniform clutter around the template, wide enough to be rejected by
        # the residual gate but not necessarily outside the bounding box.
        extra = np.column_stack(
            (rng.uniform(-25.0, 25.0, ghosts), rng.uniform(-14.0, 14.0, ghosts))
        )
        cones = np.vstack((cones, extra))
    return cones, e, c_right, c_left, o


def _car_position(e, o, distance=14.4):
    return o - e * distance


def test_fit_clean_template():
    cones, e, c_right, c_left, o = _template(noise=0.0)
    fit, diag = fit_skidpad(cones, e, _car_position(e, o))
    assert fit is not None, diag
    assert np.linalg.norm(fit.c_right - c_right) <= 0.3
    assert np.linalg.norm(fit.c_left - c_left) <= 0.3
    assert abs(fit.R - R) <= 0.2
    assert abs(fit.o[0] - o[0]) <= 0.3
    assert np.linalg.norm(fit.c_right - c_right) < np.linalg.norm(
        fit.c_right - c_left
    )
    assert fit.n_cones >= 40
    assert fit.rms <= 0.35


def test_fit_noisy_with_dropouts_and_ghosts():
    cones, e, c_right, c_left, o = _template(
        noise=0.05, seed=7, drop=0.10, ghosts=5
    )
    fit, diag = fit_skidpad(cones, e, _car_position(e, o))
    assert fit is not None, diag
    assert np.linalg.norm(fit.c_right - c_right) <= 0.3
    assert np.linalg.norm(fit.c_left - c_left) <= 0.3
    assert abs(fit.R - R) <= 0.2
    assert diag["removed"] >= 5
    assert diag["rms"] <= 0.35


def test_right_circle_for_all_entry_headings():
    for entry_deg in (0.0, 90.0, 180.0, 270.0):
        cones, e, c_right, c_left, o = _template(
            entry_deg=entry_deg, noise=0.03, seed=3
        )
        fit, diag = fit_skidpad(cones, e, _car_position(e, o))
        assert fit is not None, f"entry {entry_deg}: {diag}"
        assert np.linalg.norm(fit.c_right - c_right) < 0.3
        assert np.linalg.norm(fit.c_right - c_right) < np.linalg.norm(
            fit.c_right - c_left
        )


def test_rejects_normal_track_corridor():
    # Straight trackdrive-style corridor: two parallel rows, not two rings.
    y = np.linspace(0.0, 20.0, 20)
    left = np.column_stack((-2.0 * np.ones_like(y), y))
    right = np.column_stack((2.0 * np.ones_like(y), y))
    cones = np.vstack((left, right))
    e = np.array([0.0, 1.0])
    fit, diag = fit_skidpad(cones, e, np.array([0.0, -10.0]))
    assert fit is None
    assert diag["reasons"]


def test_rejects_too_few_cones():
    cones = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
    fit, diag = fit_skidpad(cones, np.array([0.0, 1.0]))
    assert fit is None
    assert diag["reasons"]


def test_entry_start_anomaly_is_warning_not_failure():
    cones, e, c_right, c_left, o = _template(noise=0.02, seed=9)
    # Car far to the side: warning only, fit still accepted.
    fit, diag = fit_skidpad(cones, e, o + np.array([20.0, -14.4]))
    assert fit is not None, diag
    assert fit.warnings


def test_boundary_filter_excludes_start_line_orange():
    cones = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    colors = [0, 4, 2, 3]  # blue, yellow, big orange, small orange
    selected = select_boundary_cones(cones, colors)
    assert selected.shape == (2, 2)
    assert np.allclose(selected, [[0.0, 0.0], [1.0, 0.0]])
