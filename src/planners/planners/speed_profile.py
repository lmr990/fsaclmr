"""Three-pass speed profile generation over a planned path.

Port of the HUAT FSD speed planner: a lateral-acceleration limit with
lookahead curvature preview, then a forward (acceleration) pass and a
backward (braking) pass over the arc-length discretised path. Pure numpy,
no ROS dependencies, so it is directly unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SpeedProfileParams:
    speed_cap: float = 6.0          # [m/s] absolute ceiling
    max_lateral_acc: float = 5.0    # [m/s^2] cornering limit
    max_accel: float = 2.5          # [m/s^2] longitudinal acceleration limit
    max_brake: float = 4.0          # [m/s^2] braking limit (positive number)
    min_speed: float = 2.0          # [m/s] floor (except end-of-path stop)
    curvature_epsilon: float = 1e-3 # [1/m] below this curvature the point is straight
    current_speed: float = 0.0      # [m/s] seed for the first point
    curvature_lookahead_m: float = 5.0  # [m] preview window (0 = point curvature only)
    decel_to_stop_at_end: bool = True   # force the last point down to min_speed


def compute_curvatures(path: np.ndarray, smoothing_window: int = 3) -> np.ndarray:
    """Signed curvature for each point of an (N, 2) path.

    Uses the three-point circumradius formula; endpoints copy their
    neighbour. An optional moving-average smoothing (odd window >= 3,
    pass 0 or 1 to disable) suppresses per-point noise before the speed
    passes consume it.
    """
    n = len(path)
    curvatures = np.zeros(n, dtype=np.float64)
    if n < 3:
        return curvatures

    p1 = path[:-2]
    p2 = path[1:-1]
    p3 = path[2:]

    a = np.linalg.norm(p2 - p1, axis=1)
    b = np.linalg.norm(p3 - p2, axis=1)
    c = np.linalg.norm(p3 - p1, axis=1)
    denom = a * b * c
    cross = (p2[:, 0] - p1[:, 0]) * (p3[:, 1] - p1[:, 1]) - (p2[:, 1] - p1[:, 1]) * (
        p3[:, 0] - p1[:, 0]
    )

    with np.errstate(divide="ignore", invalid="ignore"):
        k = 2.0 * cross / denom
    k = np.where((denom > 1e-9) & np.isfinite(k), k, 0.0)

    curvatures[1:-1] = k
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]

    if smoothing_window >= 3:
        w = int(smoothing_window) | 1  # force odd
        kernel = np.ones(w) / w
        padded = np.pad(curvatures, (w // 2, w // 2), mode="edge")
        curvatures = np.convolve(padded, kernel, mode="valid")

    return curvatures


def compute_speed_profile(
    path: np.ndarray,
    curvatures: np.ndarray,
    params: SpeedProfileParams,
) -> np.ndarray:
    """Feasible per-point target speeds for an (N, 2) path.

    Pass 1: v_lat = sqrt(a_lat / |kappa|) with a lookahead window taking
            the max curvature ahead, clamped by speed_cap.
    Pass 2: forward pass limited by max_accel.
    Pass 3: backward pass limited by max_brake.
    """
    n = len(path)
    if n == 0:
        return np.zeros(0, dtype=np.float64)

    v_cap = max(0.0, params.speed_cap)
    a_lat = max(0.1, params.max_lateral_acc)
    a_acc = max(0.0, params.max_accel)
    a_brake = max(0.0, params.max_brake)
    kappa_eps = max(1e-6, params.curvature_epsilon)
    v_min = max(0.0, min(params.min_speed, v_cap))

    # Segment lengths between consecutive points
    if n >= 2:
        ds = np.maximum(1e-3, np.linalg.norm(np.diff(path, axis=0), axis=1))
    else:
        ds = np.array([1e-3])

    # Lookahead preview: max |kappa| within a forward window of
    # curvature_lookahead_m metres, so braking starts BEFORE the corner.
    abs_k = np.abs(curvatures)
    if params.curvature_lookahead_m > 0.0 and n >= 2:
        effective_kappa = np.empty(n, dtype=np.float64)
        for i in range(n):
            max_k = abs_k[i]
            dist_acc = 0.0
            for j in range(i, n - 1):
                dist_acc += ds[j]
                if dist_acc > params.curvature_lookahead_m:
                    break
                if abs_k[j + 1] > max_k:
                    max_k = abs_k[j + 1]
            effective_kappa[i] = max_k
    else:
        effective_kappa = abs_k

    # PASS 1: lateral acceleration ceiling (never lifted above by v_min)
    v_ref = np.minimum(v_cap, np.sqrt(a_lat / np.maximum(effective_kappa, kappa_eps)))

    # Seed with the vehicle's actual speed at the first point
    if np.isfinite(params.current_speed) and params.current_speed >= 0.0:
        v_ref[0] = min(v_cap, params.current_speed)

    if params.decel_to_stop_at_end and n >= 2:
        v_ref[n - 1] = min(v_ref[n - 1], v_min)

    # PASS 2: forward (acceleration) pass
    for i in range(1, n):
        reachable = np.sqrt(max(0.0, v_ref[i - 1] ** 2 + 2.0 * a_acc * ds[i - 1]))
        v_ref[i] = min(v_ref[i], reachable)

    # PASS 3: backward (braking) pass
    for i in range(n - 1, 0, -1):
        reachable = np.sqrt(max(0.0, v_ref[i] ** 2 + 2.0 * a_brake * ds[i - 1]))
        v_ref[i - 1] = min(v_ref[i - 1], reachable)

    # Sanitise: clamp to [0, v_cap], never below zero, no NaN
    v_ref = np.clip(np.nan_to_num(v_ref, nan=0.0), 0.0, v_cap)
    return v_ref
