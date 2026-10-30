"""Robust FSAC skidpad template fitting from a cone map.

Pure numpy/scipy, no ROS imports. The template is two concentric-circle
pairs (inner/outer cone rows) sharing one centre per circle; the two circle
centres are separated by ~2R and their common tangent at the intersection
is the entry heading.

The fit is deliberately geometry-only: cone colours are not used, so ghost
cones and misclassified colours are handled by residual rejection instead
of colour gates.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import least_squares

# driverless_msgs/Cone.msg constants. Skidpad boundary cones are blue/yellow;
# orange cones mark the start/finish line and would otherwise be rejected as
# outliers, so the node feeds only these two colours to the template fit.
CONE_BLUE = 0
CONE_YELLOW = 4
BOUNDARY_COLORS = (CONE_BLUE, CONE_YELLOW)


def select_boundary_cones(
    cones: Sequence[Sequence[float]], colors: Sequence[int]
) -> np.ndarray:
    """Return only blue/yellow cone positions (template boundary rows)."""
    pts = np.asarray(cones, dtype=float).reshape(-1, 2)
    cols = np.asarray(colors, dtype=int).reshape(-1)
    if len(cols) != len(pts):
        raise ValueError("cones and colors length mismatch")
    mask = np.isin(cols, BOUNDARY_COLORS)
    return pts[mask]


def _unit(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float).reshape(2)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("zero-length direction")
    return vector / norm


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _right_of(heading: np.ndarray) -> np.ndarray:
    return np.array([heading[1], -heading[0]])


def _kasa_circle(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """Algebraic (Kasa) circle fit; used only as an initial centre."""
    x = points[:, 0]
    y = points[:, 1]
    a = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(points))))
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return points.mean(axis=0), float("nan")
    cx, cy, c = float(sol[0]), float(sol[1]), float(sol[2])
    r2 = c + cx * cx + cy * cy
    return np.array([cx, cy]), float(np.sqrt(max(r2, 0.0)))


def _one_d_two_means(values: np.ndarray, iterations: int = 30) -> Tuple[float, float]:
    """Two-cluster 1D k-means; returns (low, high) cluster centres."""
    values = np.asarray(values, dtype=float)
    if len(values) < 2:
        return float(values[0]), float(values[0])
    lo, hi = np.percentile(values, [20.0, 80.0])
    if hi - lo < 1e-9:
        lo, hi = values.min(), values.max()
        if hi - lo < 1e-9:
            return float(lo), float(hi)
    for _ in range(iterations):
        assign_lo = np.abs(values - lo) <= np.abs(values - hi)
        if not np.any(assign_lo):
            assign_lo[np.argmin(values)] = True
        if np.all(assign_lo):
            assign_lo[np.argmax(values)] = False
        new_lo = float(np.mean(values[assign_lo]))
        new_hi = float(np.mean(values[~assign_lo]))
        if abs(new_lo - lo) < 1e-6 and abs(new_hi - hi) < 1e-6:
            lo, hi = new_lo, new_hi
            break
        lo, hi = new_lo, new_hi
    return float(min(lo, hi)), float(max(lo, hi))


@dataclass
class FitParams:
    min_cones_total: int = 40
    min_cones_per_circle: int = 16
    rms_max: float = 0.35
    p90_max: float = 0.5
    centre_dist_range: Tuple[float, float] = (16.0, 20.5)
    radius_range: Tuple[float, float] = (7.5, 11.0)
    ring_width_range: Tuple[float, float] = (2.4, 3.6)
    outlier_dist: float = 0.6
    max_outlier_frac: float = 0.20
    max_radius_diff: float = 0.5
    max_centre_radius_mismatch: float = 0.5
    max_alignment_deg: float = 15.0
    # Entry start sanity (warning only, per spec).
    entry_along_range: Tuple[float, float] = (3.0, 30.0)
    entry_lateral_max: float = 2.5


@dataclass
class TwoRadiusFit:
    centre: np.ndarray
    r_in: float
    r_out: float
    residuals: np.ndarray       # per input point, min distance to either row
    inlier_mask: np.ndarray
    n_points: int

    @property
    def R(self) -> float:
        return 0.5 * (self.r_in + self.r_out)

    @property
    def ring_width(self) -> float:
        return self.r_out - self.r_in

    @property
    def rms(self) -> float:
        r = self.residuals[self.inlier_mask]
        return float(np.sqrt(np.mean(r * r))) if len(r) else float("inf")

    @property
    def p90(self) -> float:
        r = self.residuals[self.inlier_mask]
        return float(np.percentile(r, 90.0)) if len(r) else float("inf")


@dataclass
class SkidpadFit:
    c_left: np.ndarray
    c_right: np.ndarray
    o: np.ndarray
    R: float
    r_in: float
    r_out: float
    ring_width: float
    centre_dist: float
    e: np.ndarray
    rms: float
    p90: float
    n_cones: int          # inliers
    n_left: int
    n_right: int
    removed: int
    alignment_deg: float
    entry_along: Optional[float]
    entry_lateral: Optional[float]
    warnings: List[str] = field(default_factory=list)


def _fit_two_radius(points: np.ndarray, params: FitParams) -> Optional[TwoRadiusFit]:
    """Fit one circle's inner/outer rows with a shared centre."""
    if len(points) < 4:
        return None

    centre, _ = _kasa_circle(points)
    if not np.all(np.isfinite(centre)):
        centre = points.mean(axis=0)
    dist = np.linalg.norm(points - centre[None, :], axis=1)
    r_in, r_out = _one_d_two_means(dist)
    if r_out - r_in < 1e-6:
        r_in, r_out = float(np.min(dist)), float(np.max(dist))

    assign = np.abs(dist - r_in) <= np.abs(dist - r_out)
    best = None
    for _ in range(8):
        if not np.any(assign) or np.all(assign):
            # Degenerate split: split by the median distance.
            order = np.argsort(dist)
            assign = np.zeros(len(dist), dtype=bool)
            assign[order[: len(order) // 2]] = True
            r_in = float(np.mean(dist[assign]))
            r_out = float(np.mean(dist[~assign]))

        def residual(theta: np.ndarray) -> np.ndarray:
            cx, cy, ri, ro = theta
            dd = np.linalg.norm(points - np.array([cx, cy])[None, :], axis=1)
            rr = np.where(assign, ri, ro)
            return dd - rr

        x0 = np.array([centre[0], centre[1], r_in, r_out], dtype=float)
        try:
            result = least_squares(
                residual,
                x0,
                bounds=(
                    np.array([-np.inf, -np.inf, 0.1, 0.1]),
                    np.array([np.inf, np.inf, 50.0, 50.0]),
                ),
                loss="soft_l1",
                f_scale=0.1,
                max_nfev=200,
            )
        except (ValueError, np.linalg.LinAlgError):
            break
        new_centre = np.array([result.x[0], result.x[1]], dtype=float)
        ri, ro = float(result.x[2]), float(result.x[3])
        if ri > ro:
            ri, ro = ro, ri
        new_dist = np.linalg.norm(points - new_centre[None, :], axis=1)
        new_assign = np.abs(new_dist - ri) <= np.abs(new_dist - ro)
        converged = (
            np.array_equal(new_assign, assign)
            and np.linalg.norm(new_centre - centre) < 1e-4
            and abs(ri - r_in) < 1e-4
            and abs(ro - r_out) < 1e-4
        )
        centre, r_in, r_out, assign = new_centre, ri, ro, new_assign
        best = (centre, r_in, r_out, assign)
        if converged:
            break

    if best is None:
        best = (centre, r_in, r_out, assign)
    centre, r_in, r_out, assign = best
    dist = np.linalg.norm(points - centre[None, :], axis=1)
    residuals = np.minimum(np.abs(dist - r_in), np.abs(dist - r_out))
    return TwoRadiusFit(
        centre=centre,
        r_in=float(min(r_in, r_out)),
        r_out=float(max(r_in, r_out)),
        residuals=residuals,
        inlier_mask=residuals <= params.outlier_dist,
        n_points=len(points),
    )


def _split_along_axis(
    cones: np.ndarray, e: np.ndarray
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Split cones into the two circle groups along the centre line.

    The centre line is perpendicular to the entry heading. A 1D two-means
    split is used instead of a largest-gap split: neighbouring cones on one
    ring can be farther apart in projection than the two circles are, so the
    largest gap is not reliably the circle boundary.
    """
    axis = _right_of(e)
    projection = cones @ axis
    if len(projection) < 4:
        return None
    lo, hi = _one_d_two_means(projection)
    assign = np.abs(projection - lo) <= np.abs(projection - hi)
    first = cones[assign]
    second = cones[~assign]
    if len(first) < 4 or len(second) < 4:
        return None
    return first, second


def fit_skidpad(
    cones: Sequence[Sequence[float]],
    entry_heading: Sequence[float],
    car_position: Optional[Sequence[float]] = None,
    params: Optional[FitParams] = None,
) -> Tuple[Optional[SkidpadFit], Dict[str, object]]:
    """Fit the two-circle skidpad template.

    Returns ``(fit, diagnostics)``. When any quality gate fails, ``fit`` is
    None and ``diagnostics['reasons']`` lists the failures; the caller logs
    them at 1 Hz and waits for a better map.
    """
    p = params or FitParams()
    diag: Dict[str, object] = {"reasons": [], "warnings": []}
    reasons: List[str] = diag["reasons"]  # type: ignore[assignment]
    warnings: List[str] = diag["warnings"]  # type: ignore[assignment]

    cones = np.asarray(cones, dtype=float).reshape(-1, 2)
    cones = cones[np.all(np.isfinite(cones), axis=1)]
    diag["n_input"] = int(len(cones))
    if len(cones) < 8:
        reasons.append(f"too few cone positions ({len(cones)})")
        return None, diag

    e = _unit(entry_heading)
    split = _split_along_axis(cones, e)
    if split is None:
        reasons.append("could not split cones into two circle groups")
        return None, diag
    g1, g2 = split
    diag["n_groups"] = [int(len(g1)), int(len(g2))]
    if min(len(g1), len(g2)) < p.min_cones_per_circle:
        reasons.append(
            f"group sizes {len(g1)}/{len(g2)} < {p.min_cones_per_circle}"
        )
        return None, diag

    fits = [_fit_two_radius(g1, p), _fit_two_radius(g2, p)]
    if any(f is None for f in fits):
        reasons.append("two-radius fit failed")
        return None, diag
    f1, f2 = fits  # type: ignore[misc]

    # Robust refit: drop residuals > outlier_dist, cap the removed fraction,
    # refit once and keep only inliers.
    groups = [g1, g2]
    for _pass in range(2):
        residual = np.concatenate([f.residuals for f in (f1, f2)])
        mask = residual <= p.outlier_dist
        removed = int(np.count_nonzero(~mask))
        if removed > p.max_outlier_frac * len(residual):
            reasons.append(
                f"outliers {removed}/{len(residual)} exceed "
                f"{p.max_outlier_frac:.0%}"
            )
            return None, diag
        starts = [0, len(f1.residuals)]
        new_fits = []
        for k, group in enumerate(groups):
            lo, hi = starts[k], starts[k] + len(group)
            keep = mask[lo:hi]
            if np.count_nonzero(keep) < 4:
                reasons.append("too few inliers after robust refit")
                return None, diag
            refit = _fit_two_radius(group[keep], p)
            if refit is None:
                reasons.append("robust refit failed")
                return None, diag
            # Keep the full-group residual for the next outlier pass, but
            # evaluate the fit on all points of the group.
            dist = np.linalg.norm(
                group - refit.centre[None, :], axis=1
            )
            refit.residuals = np.minimum(
                np.abs(dist - refit.r_in), np.abs(dist - refit.r_out)
            )
            refit.inlier_mask = refit.residuals <= p.outlier_dist
            new_fits.append(refit)
        f1, f2 = new_fits

    # Final inlier sets and quality numbers.
    inliers = []
    for f, group in ((f1, g1), (f2, g2)):
        keep = f.inlier_mask
        if np.count_nonzero(keep) < p.min_cones_per_circle:
            reasons.append(
                f"circle group has only {np.count_nonzero(keep)} inliers"
            )
            return None, diag
        inliers.append(group[keep])
    n_inliers = len(inliers[0]) + len(inliers[1])
    diag["n_inliers"] = n_inliers
    diag["removed"] = int(len(cones) - n_inliers)
    if n_inliers < p.min_cones_total:
        reasons.append(f"inlier cones {n_inliers} < {p.min_cones_total}")
        return None, diag

    all_res = np.concatenate(
        [f.residuals[f.inlier_mask] for f in (f1, f2)]
    )
    rms = float(np.sqrt(np.mean(all_res * all_res)))
    p90 = float(np.percentile(all_res, 90.0))
    diag["rms"] = rms
    diag["p90"] = p90
    if rms > p.rms_max:
        reasons.append(f"fit RMS {rms:.3f} > {p.rms_max}")
    if p90 > p.p90_max:
        reasons.append(f"fit p90 {p90:.3f} > {p.p90_max}")

    c1, c2 = f1.centre, f2.centre
    R1, R2 = f1.R, f2.R
    diag["R_group"] = [R1, R2]
    if abs(R1 - R2) > p.max_radius_diff:
        reasons.append(f"circle radii differ by {abs(R1 - R2):.3f} m")
    R = 0.5 * (R1 + R2)
    r_in = 0.5 * (f1.r_in + f2.r_in)
    r_out = 0.5 * (f1.r_out + f2.r_out)
    ring_width = r_out - r_in
    centre_dist = float(np.linalg.norm(c1 - c2))
    diag.update(
        {
            "R": R,
            "r_in": r_in,
            "r_out": r_out,
            "ring_width": ring_width,
            "centre_dist": centre_dist,
        }
    )
    if not (p.centre_dist_range[0] <= centre_dist <= p.centre_dist_range[1]):
        reasons.append(f"centre distance {centre_dist:.2f} outside range")
    if not (p.radius_range[0] <= R <= p.radius_range[1]):
        reasons.append(f"R {R:.2f} outside range")
    if not (p.ring_width_range[0] <= ring_width <= p.ring_width_range[1]):
        reasons.append(f"ring width {ring_width:.2f} outside range")
    if abs(0.5 * centre_dist - R) > p.max_centre_radius_mismatch:
        reasons.append(
            f"centre distance/2 {centre_dist / 2:.2f} vs R {R:.2f} mismatch"
        )

    o = 0.5 * (c1 + c2)
    # Right circle: cross(e, c - o) < 0 (FSAC convention, z up).
    if _cross(e, c1 - o) < 0.0 <= _cross(e, c2 - o):
        c_right, c_left = c1, c2
    elif _cross(e, c2 - o) < 0.0 <= _cross(e, c1 - o):
        c_right, c_left = c2, c1
    else:
        reasons.append("cannot assign left/right circles from entry heading")
        return None, diag

    fitted_axis = _unit(c_right - c_left)
    gen_axis = _right_of(e)
    alignment = float(
        np.degrees(np.arccos(np.clip(abs(np.dot(fitted_axis, gen_axis)), -1.0, 1.0)))
    )
    diag["alignment_deg"] = alignment
    if alignment > p.max_alignment_deg:
        warnings.append(f"fitted template rotated {alignment:.1f} deg vs heading")

    entry_along = entry_lateral = None
    if car_position is not None:
        car = np.asarray(car_position, dtype=float).reshape(2)
        rel = car - o
        entry_along = float(np.dot(rel, -e))
        entry_lateral = float(abs(_cross(e, rel)))
        diag["entry_along"] = entry_along
        diag["entry_lateral"] = entry_lateral
        if not (p.entry_along_range[0] <= entry_along <= p.entry_along_range[1]):
            warnings.append(
                f"car entry distance {entry_along:.2f} m outside "
                f"{p.entry_along_range}"
            )
        if entry_lateral > p.entry_lateral_max:
            warnings.append(f"car lateral entry offset {entry_lateral:.2f} m")

    if reasons:
        return None, diag

    fit = SkidpadFit(
        c_left=c_left,
        c_right=c_right,
        o=o,
        R=float(R),
        r_in=float(r_in),
        r_out=float(r_out),
        ring_width=float(ring_width),
        centre_dist=float(centre_dist),
        e=e,
        rms=rms,
        p90=p90,
        n_cones=n_inliers,
        n_left=int(np.count_nonzero(f1.inlier_mask)),
        n_right=int(np.count_nonzero(f2.inlier_mask)),
        removed=int(len(cones) - n_inliers),
        alignment_deg=alignment,
        entry_along=entry_along,
        entry_lateral=entry_lateral,
        warnings=list(warnings),
    )
    # Fix left/right inlier counts after assignment.
    if np.allclose(c_right, c1):
        fit.n_left, fit.n_right = int(np.count_nonzero(f2.inlier_mask)), int(
            np.count_nonzero(f1.inlier_mask)
        )
    else:
        fit.n_left, fit.n_right = int(np.count_nonzero(f1.inlier_mask)), int(
            np.count_nonzero(f2.inlier_mask)
        )
    return fit, diag
