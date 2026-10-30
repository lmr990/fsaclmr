"""Unit tests for the skidpad monotonic progress tracker (no ROS)."""

import math

import numpy as np

from planners.skidpad_geometry import generate_skidpad_path
from planners.skidpad_progress import (
    STATE_DRIVING,
    STATE_FAULT,
    STATE_FINISHED,
    ProgressConfig,
    SkidpadProgress,
)

R = 9.125
DS = 0.25


def _path():
    e = np.array([0.0, 1.0])
    o = np.zeros(2)
    right = np.array([1.0, 0.0])
    return generate_skidpad_path(o + R * right, o - R * right, R, e, 15.0, 20.0, DS)


def _pose(path, s):
    x = float(np.interp(s, path.arc_length, path.points[:, 0]))
    y = float(np.interp(s, path.arc_length, path.points[:, 1]))
    return x, y


def test_progress_is_monotonic_and_follows_segments():
    path = _path()
    prog = SkidpadProgress(path, ProgressConfig())
    start_s = 2.0
    x, y = _pose(path, start_s)
    assert prog.initialize(x, y)
    assert prog.state == STATE_DRIVING

    rng = np.random.default_rng(11)
    seen = []
    last_s = prog.s_prog
    for s in np.arange(start_s, path.total_length, 0.1):
        x, y = _pose(path, s)
        x += rng.normal(0.0, 0.1)
        y += rng.normal(0.0, 0.1)
        upd = prog.update(x, y, now=float(s))
        assert upd.s_prog >= last_s - 1e-9, "progress went backwards"
        last_s = upd.s_prog
        if not seen or seen[-1] != upd.segment:
            seen.append(upd.segment)
        if upd.window is not None:
            i0, i1 = upd.window
            assert path.arc_length[i1] >= min(
                upd.s_prog + 8.0, path.total_length
            ) - 1e-6
        if upd.state == STATE_FINISHED:
            break

    assert seen == [
        "ENTRY",
        "RIGHT_1",
        "RIGHT_2",
        "LEFT_1",
        "LEFT_2",
        "EXIT",
    ]
    assert prog.state == STATE_FINISHED
    assert prog.window_indices() is None


def test_off_path_fault_after_one_second():
    path = _path()
    prog = SkidpadProgress(path, ProgressConfig(off_path_dist=2.0, off_path_time=1.0))
    x, y = _pose(path, 2.0)
    assert prog.initialize(x, y)
    # 3 m to the left of the entry straight (entry heading +y).
    upd = None
    for t in np.arange(0.0, 1.2, 0.1):
        upd = prog.update(x - 3.0, y, now=float(t))
    assert upd is not None
    assert upd.state == STATE_FAULT
    assert upd.window is None


def test_does_not_jump_ahead_at_intersection():
    path = _path()
    prog = SkidpadProgress(path, ProgressConfig())
    # Start just before the first intersection (end of ENTRY).
    s = path.arc_length[path.index_range("ENTRY")[1]] - 0.5
    x, y = _pose(path, s)
    assert prog.initialize(x, y)
    # Feed the exact intersection point repeatedly; the local window must
    # keep progress at/just after the intersection, not jump a full circle.
    o = path.o
    for k in range(20):
        upd = prog.update(float(o[0]), float(o[1]), now=float(k) * 0.1)
        assert upd.s_prog <= s + 9.0
    assert prog.path.segment_at(prog.s_prog) in ("RIGHT_1", "RIGHT_2")
