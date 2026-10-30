"""Analytic FSAC skidpad (figure-eight) path generation.

Pure numpy, no ROS imports: the geometry is unit-testable in isolation.

Template (FSAC 2025 §6.4):
  * two tangent circles of equal centreline radius ``R``;
  * their centres lie on the line perpendicular to the entry heading ``e``;
  * the intersection ``o`` is the midpoint of the centres and the common
    tangent there is ``e``;
  * ENTRY runs along ``e`` into ``o``, then right circle twice, left circle
    twice, then EXIT along ``e``.

The generated path is a polyline with arc-length spacing ``ds``. Points at
segment boundaries are not duplicated: each segment includes its start and
excludes its end, except EXIT which includes its endpoint. The intersection
therefore appears once at the start of each circle/EXIT segment, which is
intentional (the car passes it five times).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

from planners.speed_profile import (
    SpeedProfileParams,
    compute_curvatures,
    compute_speed_profile,
)

SEGMENT_ORDER: Tuple[str, ...] = (
    "ENTRY",
    "RIGHT_1",
    "RIGHT_2",
    "LEFT_1",
    "LEFT_2",
    "EXIT",
)

# Fixed lap numbering used by planning/skidpad_state (FSAC procedure order).
SEGMENT_LAP: Dict[str, int] = {
    "ENTRY": 0,
    "RIGHT_1": 1,
    "RIGHT_2": 2,
    "LEFT_1": 3,
    "LEFT_2": 4,
    "EXIT": 5,
}


def _unit(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float).reshape(2)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("zero-length direction vector")
    return vector / norm


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _right_of(heading: np.ndarray) -> np.ndarray:
    """Unit vector 90 deg clockwise from ``heading`` (right-hand side)."""
    return np.array([heading[1], -heading[0]])


def _sample_line(
    start: np.ndarray,
    direction: np.ndarray,
    length: float,
    ds: float,
    include_end: bool,
) -> np.ndarray:
    if length <= 0.0:
        return np.array([start], dtype=float)
    n = max(1, int(np.ceil(length / ds)))
    t = np.arange(n, dtype=float) * ds
    if include_end:
        if t[-1] < length - 1e-9:
            t = np.append(t, length)
        else:
            t[-1] = length
    else:
        t = t[t < length - 1e-9]
    return start[None, :] + t[:, None] * direction[None, :]


def _sample_circle(
    centre: np.ndarray,
    radius: float,
    theta0: float,
    winding: int,
    ds: float,
    include_end: bool,
) -> np.ndarray:
    """Sample a full circle; winding +1 = CCW, -1 = CW."""
    dtheta = ds / max(radius, 1e-9)
    n = max(3, int(np.ceil(2.0 * np.pi / dtheta)))
    dtheta = 2.0 * np.pi / n  # exact full turn, step <= ds
    theta = theta0 + winding * dtheta * np.arange(n + 1, dtype=float)
    points = centre[None, :] + radius * np.column_stack(
        (np.cos(theta), np.sin(theta))
    )
    return points if include_end else points[:-1]


@dataclass(frozen=True)
class SkidpadPath:
    points: np.ndarray          # (N, 2) in track frame
    arc_length: np.ndarray      # (N,) cumulative length [m]
    segments: Tuple[str, ...]   # per-point segment label
    total_length: float         # polyline length [m]
    nominal_length: float       # entry + 4*2*pi*R + exit [m]
    entry_len: float
    exit_len: float
    o: np.ndarray               # intersection of the two circles
    c_right: np.ndarray         # generated right-circle centre
    c_left: np.ndarray          # generated left-circle centre
    R: float                    # generated centreline radius
    e: np.ndarray               # unit entry/exit heading
    alignment_deg: float        # fitted-centre line vs generated orientation

    def index_range(self, name: str) -> Tuple[int, int]:
        """[first, last] index of a segment (inclusive)."""
        idx = [i for i, s in enumerate(self.segments) if s == name]
        if not idx:
            raise KeyError(name)
        return idx[0], idx[-1]

    def segment_at(self, s: float) -> str:
        """Segment label at arc length ``s`` (clamped to the path)."""
        i = int(np.clip(np.searchsorted(self.arc_length, s), 0, len(self.points) - 1))
        return self.segments[i]


def generate_skidpad_path(
    c_right: Sequence[float],
    c_left: Sequence[float],
    R: float,
    e: Sequence[float],
    entry_len: float,
    exit_len: float,
    ds: float = 0.25,
) -> SkidpadPath:
    """Build the analytic figure-eight path from a fitted template.

    The fitted centres only supply the intersection ``o`` and the radius
    ``R``; the generated centres are placed symmetrically at ``o +/- R *
    right(e)`` so the two circles are exactly tangent at ``o`` and their
    common tangent there is exactly ``e``. This also removes small fit
    rotations (the resulting angular correction is reported in
    ``alignment_deg``).
    """
    if R <= 0.0:
        raise ValueError("R must be positive")
    if ds <= 0.0:
        raise ValueError("ds must be positive")
    if entry_len <= 0.0 or exit_len <= 0.0:
        raise ValueError("entry_len and exit_len must be positive")

    c_right = np.asarray(c_right, dtype=float).reshape(2)
    c_left = np.asarray(c_left, dtype=float).reshape(2)
    e = _unit(e)
    o = 0.5 * (c_right + c_left)

    right_dir = _right_of(e)
    # Preserve the fitted left/right assignment: the right centre must lie on
    # the right-hand side of the entry heading.
    if float(np.dot(c_right - o, right_dir)) < 0.0:
        right_dir = -right_dir
    c_right_gen = o + R * right_dir
    c_left_gen = o - R * right_dir

    fitted_axis = _unit(c_right - c_left)
    # Angle between the fitted centre line and the generated one (both are
    # lines, so compare against the generated axis which is +/-right_dir).
    gen_axis = _unit(c_right_gen - c_left_gen)
    alignment = float(
        np.degrees(np.arccos(np.clip(abs(np.dot(fitted_axis, gen_axis)), -1.0, 1.0)))
    )

    entry = _sample_line(o - e * entry_len, e, entry_len, ds, include_end=False)

    points_parts: List[np.ndarray] = [entry]
    labels: List[str] = ["ENTRY"] * len(entry)

    for name, centre in (
        ("RIGHT_1", c_right_gen),
        ("RIGHT_2", c_right_gen),
        ("LEFT_1", c_left_gen),
        ("LEFT_2", c_left_gen),
    ):
        r0 = o - centre
        if abs(np.linalg.norm(r0) - R) > 1e-6:
            raise ValueError("internal error: centre/intersection radius mismatch")
        theta0 = float(np.arctan2(r0[1], r0[0]))
        winding = 1 if _cross(r0, e) > 0.0 else -1
        pts = _sample_circle(centre, R, theta0, winding, ds, include_end=False)
        # The first sample must be exactly the intersection.
        if np.linalg.norm(pts[0] - o) > 1e-6:
            raise ValueError("internal error: circle does not start at intersection")
        # The first chord must leave along the entry heading.
        tangent = _unit(pts[1] - pts[0])
        if float(np.dot(tangent, e)) < 0.999:
            raise ValueError("internal error: circle tangent does not match entry heading")
        points_parts.append(pts)
        labels.extend([name] * len(pts))

    exit_pts = _sample_line(o, e, exit_len, ds, include_end=True)
    points_parts.append(exit_pts)
    labels.extend(["EXIT"] * len(exit_pts))

    points = np.vstack(points_parts)
    arc = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )
    nominal = entry_len + 4.0 * 2.0 * np.pi * R + exit_len

    return SkidpadPath(
        points=points,
        arc_length=arc,
        segments=tuple(labels),
        total_length=float(arc[-1]),
        nominal_length=float(nominal),
        entry_len=float(entry_len),
        exit_len=float(exit_len),
        o=o,
        c_right=c_right_gen,
        c_left=c_left_gen,
        R=float(R),
        e=e,
        alignment_deg=alignment,
    )


def _heading_degrees(delta: np.ndarray) -> float:
    return float(np.degrees(np.arctan2(delta[1], delta[0])))


def _angle_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return float(d)


def validate_skidpad_path(
    path: SkidpadPath,
    ds: float = 0.25,
    length_tol: float = 0.02,
    tangent_tol: float = 0.99,
    max_heading_jump_deg: float = 2.0,
    curvature_tol: float = 0.05,
    geometry_tol: float = 0.05,
) -> List[str]:
    """Self-check the generated geometry; returns a list of human-readable errors."""
    errors: List[str] = []
    if path.total_length > path.nominal_length * (1.0 + length_tol):
        errors.append(
            f"total length {path.total_length:.3f} exceeds nominal "
            f"{path.nominal_length:.3f}"
        )
    if path.total_length < path.nominal_length * (1.0 - length_tol):
        errors.append(
            f"total length {path.total_length:.3f} below nominal "
            f"{path.nominal_length:.3f}"
        )

    # Segment order (first occurrence of each label must follow the rule).
    seen: List[str] = []
    for label in path.segments:
        if not seen or seen[-1] != label:
            seen.append(label)
    if tuple(seen) != SEGMENT_ORDER:
        errors.append(f"segment order {seen} != {list(SEGMENT_ORDER)}")

    # Continuity and heading jump.
    deltas = np.diff(path.points, axis=0)
    steps = np.linalg.norm(deltas, axis=1)
    if np.any(steps <= 1e-9):
        errors.append("duplicate consecutive path point")
    if np.any(steps > 1.5 * ds + 1e-9):
        errors.append(f"path gap > 1.5*ds (max {steps.max():.3f} m)")
    yaws = np.degrees(np.arctan2(deltas[:, 1], deltas[:, 0]))
    jumps = np.abs((np.diff(yaws) + 180.0) % 360.0 - 180.0)
    if len(jumps) and jumps.max() > max_heading_jump_deg:
        errors.append(
            f"heading jump {jumps.max():.2f} deg at index "
            f"{int(np.argmax(jumps)) + 1} > {max_heading_jump_deg} deg"
        )

    # Per-circle arc length, tangent sign and mean curvature.
    curvatures = compute_curvatures(path.points)
    ideal_curvature = 1.0 / path.R
    for name in ("RIGHT_1", "RIGHT_2", "LEFT_1", "LEFT_2"):
        lo, hi = path.index_range(name)
        seg = path.points[lo : hi + 1]
        seg_len = float(np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1)))
        expected = 2.0 * np.pi * path.R
        # One sampling step is not in the segment (the closure point starts
        # the next segment), so the tolerance covers that.
        if abs(seg_len - expected) > max(0.01 * expected, 2.0 * ds):
            errors.append(f"{name} arc length {seg_len:.3f} != {expected:.3f}")

        tangent = _unit(seg[1] - seg[0])
        if float(np.dot(tangent, path.e)) < tangent_tol:
            errors.append(f"{name} start tangent not aligned with entry heading")

        centre = path.c_right if name.startswith("RIGHT") else path.c_left
        sign = -1.0 if name.startswith("RIGHT") else 1.0
        inner = curvatures[lo + 3 : hi - 2]
        if len(inner) < 5:
            errors.append(f"{name} too short for curvature check")
        else:
            mean_k = float(np.mean(inner))
            if abs(mean_k - sign * ideal_curvature) > curvature_tol * ideal_curvature:
                errors.append(
                    f"{name} mean curvature {mean_k:.5f} != "
                    f"{sign * ideal_curvature:.5f}"
                )
            radii = np.linalg.norm(seg - centre[None, :], axis=1)
            if np.max(np.abs(radii - path.R)) > geometry_tol:
                errors.append(f"{name} point radius off by > {geometry_tol} m")

    # Entry/exit straight lines.
    for name, start, direction in (
        ("ENTRY", path.o - path.e * path.entry_len, path.e),
        ("EXIT", path.o, path.e),
    ):
        lo, hi = path.index_range(name)
        seg = path.points[lo : hi + 1]
        rel = seg - start[None, :]
        lateral = np.abs(rel[:, 0] * direction[1] - rel[:, 1] * direction[0])
        along = rel @ direction
        if lateral.max() > geometry_tol:
            errors.append(f"{name} lateral deviation {lateral.max():.3f} m")
        if along.min() < -geometry_tol or along.max() > (
            path.entry_len if name == "ENTRY" else path.exit_len
        ) + geometry_tol:
            errors.append(f"{name} samples outside its straight segment")

    return errors


def compute_skidpad_speeds(
    path: SkidpadPath,
    speed_cap: float,
    params: SpeedProfileParams | None = None,
    smoothing_window: int = 3,
    stop_len: float = 4.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Curvatures and target speeds for the full path.

    The shared three-pass profile is used unchanged (library defaults) with
    only ``speed_cap`` overridden. A final backward brake pass forces the
    EXIT tail to a full stop at the last point; the library's
    ``decel_to_stop_at_end`` only pulls the last point down to ``min_speed``.
    """
    base = params if params is not None else SpeedProfileParams()
    base_kwargs = vars(base).copy()
    base_kwargs["speed_cap"] = float(speed_cap)
    base_kwargs["decel_to_stop_at_end"] = True
    prof_params = SpeedProfileParams(**base_kwargs)

    curvatures = compute_curvatures(path.points, smoothing_window)
    speeds = compute_speed_profile(path.points, curvatures, prof_params)

    # Smooth stop over the final ``stop_len`` metres (and enforce it as a
    # global backward brake constraint, which is a no-op far from the end).
    speeds = speeds.copy()
    speeds[-1] = 0.0
    ds = np.linalg.norm(np.diff(path.points, axis=0), axis=1)
    brake = max(0.0, float(prof_params.max_brake))
    for i in range(len(speeds) - 2, -1, -1):
        reachable = np.sqrt(max(0.0, speeds[i + 1] ** 2 + 2.0 * brake * ds[i]))
        speeds[i] = min(speeds[i], reachable)
    # Ensure the final stop_len is monotonically decreasing even if the
    # lookahead profile was optimistic inside the last few metres.
    stop_from = path.total_length - max(0.0, stop_len)
    for i in range(len(speeds) - 2, -1, -1):
        if path.arc_length[i] < stop_from:
            break
        reachable = np.sqrt(max(0.0, speeds[i + 1] ** 2 + 2.0 * brake * ds[i]))
        speeds[i] = min(speeds[i], reachable)
    speeds = np.clip(np.nan_to_num(speeds, nan=0.0), 0.0, float(speed_cap))
    return speeds, curvatures
