"""Unit tests for the pure skidpad geometry module (no ROS/Gazebo)."""

import math

import numpy as np
import pytest

from planners.skidpad_geometry import (
    SEGMENT_LAP,
    SEGMENT_ORDER,
    compute_skidpad_speeds,
    generate_skidpad_path,
    validate_skidpad_path,
)

R = 9.125
DS = 0.25
ENTRY_LEN = 15.0
EXIT_LEN = 20.0


def _template(entry_deg: float = 90.0):
    e = np.array(
        [math.cos(math.radians(entry_deg)), math.sin(math.radians(entry_deg))]
    )
    o = np.zeros(2)
    right_dir = np.array([e[1], -e[0]])
    c_right = o + R * right_dir
    c_left = o - R * right_dir
    return c_right, c_left, e


def _path(entry_deg: float = 90.0, entry_len: float = ENTRY_LEN):
    c_right, c_left, e = _template(entry_deg)
    return generate_skidpad_path(c_right, c_left, R, e, entry_len, EXIT_LEN, DS)


def test_segment_order_and_circle_lengths():
    path = _path()
    expected = list(SEGMENT_ORDER)
    assert expected == [s for s in SEGMENT_ORDER]
    # Labels appear in rule order, contiguous.
    seen = []
    for label in path.segments:
        if not seen or seen[-1] != label:
            seen.append(label)
    assert seen == expected

    for name in ("RIGHT_1", "RIGHT_2", "LEFT_1", "LEFT_2"):
        lo, hi = path.index_range(name)
        seg = path.points[lo : hi + 1]
        seg_len = np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1))
        assert abs(seg_len - 2 * math.pi * R) <= 0.01 * 2 * math.pi * R

    assert abs(path.total_length - path.nominal_length) <= 0.02 * path.nominal_length


def test_intersection_tangent_and_curvature_signs():
    path = _path()
    # Both circles leave the intersection along the entry heading.
    for name in ("RIGHT_1", "LEFT_1"):
        lo, _ = path.index_range(name)
        tangent = path.points[lo + 1] - path.points[lo]
        tangent = tangent / np.linalg.norm(tangent)
        assert float(np.dot(tangent, path.e)) > 0.99

    kappa = np.zeros(len(path.points))
    from planners.speed_profile import compute_curvatures

    kappa = compute_curvatures(path.points)
    for name, sign in (
        ("RIGHT_1", -1.0),
        ("RIGHT_2", -1.0),
        ("LEFT_1", 1.0),
        ("LEFT_2", 1.0),
    ):
        lo, hi = path.index_range(name)
        mean_k = float(np.mean(kappa[lo + 3 : hi - 2]))
        assert abs(mean_k - sign / R) <= 0.05 / R


def test_continuity_and_heading_jumps():
    path = _path()
    deltas = np.diff(path.points, axis=0)
    steps = np.linalg.norm(deltas, axis=1)
    assert np.all(steps > 1e-9)
    assert np.all(steps <= 1.5 * DS + 1e-9)

    yaws = np.degrees(np.arctan2(deltas[:, 1], deltas[:, 0]))
    jumps = np.abs((np.diff(yaws) + 180.0) % 360.0 - 180.0)
    assert jumps.max() <= 2.0


def test_rigid_invariance_for_entry_headings():
    for entry_deg in (0.0, 37.0, 180.0, 270.0):
        path = _path(entry_deg)
        errors = validate_skidpad_path(path)
        assert errors == [], f"entry {entry_deg} deg: {errors}"
        assert path.segment_at(0.0) == "ENTRY"
        assert path.segment_at(path.total_length) == "EXIT"


def test_points_lie_on_their_ideal_geometry():
    path = _path()
    for name in ("RIGHT_1", "RIGHT_2", "LEFT_1", "LEFT_2"):
        lo, hi = path.index_range(name)
        centre = path.c_right if name.startswith("RIGHT") else path.c_left
        radii = np.linalg.norm(path.points[lo : hi + 1] - centre, axis=1)
        assert np.max(np.abs(radii - R)) <= 0.05

    for name, start in (("ENTRY", path.o - path.e * ENTRY_LEN), ("EXIT", path.o)):
        lo, hi = path.index_range(name)
        rel = path.points[lo : hi + 1] - start
        lateral = np.abs(rel[:, 0] * path.e[1] - rel[:, 1] * path.e[0])
        assert lateral.max() <= 0.05


def test_speed_profile_caps_and_stops():
    path = _path()
    speeds, curvatures = compute_skidpad_speeds(path, speed_cap=2.0)
    assert len(speeds) == len(path.points)
    assert speeds[-1] == pytest.approx(0.0, abs=1e-9)
    assert np.all(speeds >= 0.0)
    assert speeds.max() <= 2.0 + 1e-9
    assert curvatures.shape == speeds.shape
    # The final approach must be monotone non-increasing over the last few
    # metres (a genuine stop, not a step to zero at the last sample).
    last = speeds[-6:]
    assert np.all(np.diff(last) <= 1e-6)


def test_segment_lap_mapping():
    assert SEGMENT_LAP == {
        "ENTRY": 0,
        "RIGHT_1": 1,
        "RIGHT_2": 2,
        "LEFT_1": 3,
        "LEFT_2": 4,
        "EXIT": 5,
    }
