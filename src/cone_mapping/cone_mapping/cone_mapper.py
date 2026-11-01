"""Pure merge logic for the cone mapper (no ROS imports, unit-testable).

Detections arrive in a global map frame (e.g. "track") with an absolute
pose source (GNSS/INS), so mapping reduces to accumulation with known
poses. The world model is STATIC: a real cone never moves, while hands,
people walking past and spurious detections are transient. The logic
encodes that with three mechanisms:

* Tentative lifecycle — a position seen for the first time starts as a
  tentative entry (never published, short eviction age). It is promoted
  to established only after ``confirm_min_obs`` observations within
  ``confirm_radius`` of the entry, spanning at least ``confirm_span_sec``.
  A transient source either keeps moving (its observations stop being
  tight as the entry position lags behind) or stops (evicted quickly), so
  it never reaches the published map.
* Innovation gate — an established entry only folds observations closer
  than ``gate_sigma_factor`` * sigma, with sigma the per-cone standard
  deviation from ``covariance_for``. Anything farther is rejected instead
  of averaged in, so a stream of bad observations can never drag a
  confirmed cone away from its position.
* Conflict retirement — an established entry that has not accepted an
  observation for ``stale_conflict_sec`` while a same-colour established
  entry within ``conflict_radius`` is still actively observed is removed:
  the camera is seeing a cone at the new position while the old entry no
  longer receives any, i.e. the physical cone moved (kicked, pushed by a
  car). Checked periodically, so it also covers moves beyond
  merge_radius where the old entry is never even associated again.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# driverless_msgs/Cone.msg colour constants
CONE_BLUE = 0
CONE_UNKNOWN = 1
CONE_ORANGE_BIG = 2
CONE_ORANGE_SMALL = 3
CONE_YELLOW = 4


@dataclass
class MapCone:
    x: float
    y: float
    obs_count: int = 1
    color_votes: Dict[int, int] = field(default_factory=dict)
    last_seen: float = 0.0
    sim_cone_index: int = 0
    # Static-world lifecycle: every entry is born tentative and must earn
    # establishment (see process_detection). Tentative entries are never
    # published and evict after a short age.
    tentative: bool = True
    first_seen: float = 0.0
    # Observations that landed within confirm_radius of the entry position;
    # promotion needs confirm_min_obs of them spanning confirm_span_sec.
    confirm_count: int = 0

    @property
    def color(self) -> int:
        """Majority colour vote; UNKNOWN when no coloured observation yet."""
        if not self.color_votes:
            return CONE_UNKNOWN
        return max(self.color_votes.items(), key=lambda kv: kv[1])[0]


@dataclass
class MapperConfig:
    merge_radius: float = 0.5
    # Tentative -> established promotion.
    confirm_min_obs: int = 4
    confirm_span_sec: float = 1.0
    confirm_radius: float = 0.15
    # Established-entry innovation gate: reject observations farther than
    # factor * sigma instead of averaging them in. The absolute cap
    # gate_max_radius protects YOUNG established entries (obs_count small,
    # sigma large) from being dragged by a stream of consistently-offset
    # bad observations.
    gate_sigma_factor: float = 2.5
    gate_max_radius: float = 0.4
    # Periodic conflict retirement for moved cones.
    conflict_radius: float = 1.5
    stale_conflict_sec: float = 5.0


def covariance_for(obs_count: int) -> List[float]:
    """Diagonal covariance shrinking with observation count (x, y order)."""
    var = max(0.01, 0.25 / max(1, obs_count))
    return [var, 0.0, 0.0, var]


def gate_radius(cone: MapCone, gate_sigma_factor: float, gate_max_radius: float) -> float:
    """Max distance of an observation an established entry still folds.

    Same sigma as the published covariance: the uncertainty shrinks as
    the entry accumulates observations, so the gate tightens with
    evidence. The absolute cap bounds the gate for young entries (where
    sigma alone would allow observations across most of merge_radius to
    drag the position), and the 0.1 m sigma floor keeps mature entries
    open to normal depth noise.
    """
    sigma = math.sqrt(covariance_for(cone.obs_count)[0])
    return min(gate_sigma_factor * sigma, gate_max_radius)


def _fold_observation(
    cone: MapCone,
    det_x: float,
    det_y: float,
    det_color: int,
    now: float,
    det_sim_index: int = 0,
) -> None:
    """Fold one accepted detection into an entry (observation-weighted)."""
    n = cone.obs_count
    # weighted average: old position keeps weight n/(n+1), new detection 1/(n+1)
    cone.x = (cone.x * n + det_x) / (n + 1)
    cone.y = (cone.y * n + det_y) / (n + 1)
    cone.obs_count = n + 1
    cone.last_seen = now
    if det_sim_index != 0:
        cone.sim_cone_index = det_sim_index
    if det_color != CONE_UNKNOWN:
        cone.color_votes[det_color] = cone.color_votes.get(det_color, 0) + 1


def new_cone(
    det_x: float,
    det_y: float,
    det_color: int,
    now: float,
    det_sim_index: int = 0,
) -> MapCone:
    """A new entry from a first-time detection; tentative until confirmed."""
    votes: Dict[int, int] = {}
    if det_color != CONE_UNKNOWN:
        votes[det_color] = 1
    return MapCone(
        x=det_x, y=det_y, obs_count=1, color_votes=votes,
        last_seen=now, sim_cone_index=det_sim_index,
        tentative=True, first_seen=now, confirm_count=1,
    )


def process_detection(
    cones: List[MapCone],
    det_x: float,
    det_y: float,
    det_color: int,
    now: float,
    cfg: MapperConfig,
    det_sim_index: int = 0,
) -> str:
    """Fold one detection into the map under the static-world rules.

    Entries within merge_radius are visited nearest-first: a tentative
    entry always accepts (and counts the observation toward promotion if
    it landed tight AND matches the entry's colour), an established entry
    accepts only within its innovation gate. Returns the disposition:

    * "created"  — nothing within merge_radius; new tentative entry.
    * "merged"   — folded into a tentative entry or an accepting
      established entry.
    * "promoted" — the fold promoted a tentative entry to established.
    * "gated"    — established entries within merge_radius all rejected
      the observation (innovation gate); a new tentative entry is created
      at the observed position instead. If the source is real and
      persistent it will confirm there and the periodic conflict
      retirement will remove the starved established entry; if transient
      it ages out with the tentative entry.
    """
    merge_sq = cfg.merge_radius * cfg.merge_radius
    near: List[Tuple[float, MapCone]] = []
    for c in cones:
        d_sq = (c.x - det_x) ** 2 + (c.y - det_y) ** 2
        if d_sq <= merge_sq:
            near.append((d_sq, c))
    near.sort(key=lambda e: e[0])

    gated = False
    for d_sq, c in near:
        if c.tentative:
            # A tight, colour-matched observation counts toward promotion.
            # A source that keeps moving (a walking person) stops producing
            # tight observations as the entry position lags behind it, so
            # its confirm_count freezes below the threshold; a source whose
            # colour flips (misclassification) pauses confirmation too.
            tight = d_sq <= cfg.confirm_radius * cfg.confirm_radius
            if tight and (c.color == CONE_UNKNOWN or c.color == det_color):
                c.confirm_count += 1
            _fold_observation(c, det_x, det_y, det_color, now, det_sim_index)
            if (
                tight
                and c.confirm_count >= cfg.confirm_min_obs
                and (c.last_seen - c.first_seen) >= cfg.confirm_span_sec
            ):
                c.tentative = False
                return "promoted"
            return "merged"
        # Established entry: innovation gate decides.
        if d_sq <= gate_radius(c, cfg.gate_sigma_factor, cfg.gate_max_radius) ** 2:
            _fold_observation(c, det_x, det_y, det_color, now, det_sim_index)
            return "merged"
        gated = True

    if gated:
        # Rejected by the gate: nothing is averaged into the established
        # entry. A tentative is started at the observed position —
        # persistent truth confirms there, transient noise dies there.
        cones.append(new_cone(det_x, det_y, det_color, now, det_sim_index))
        return "gated"

    cones.append(new_cone(det_x, det_y, det_color, now, det_sim_index))
    return "created"


def retire_superseded(cones: List[MapCone], now: float, cfg: MapperConfig) -> int:
    """Periodic conflict retirement: remove entries of moved physical cones.

    A physical cone that moved leaves its old entry starved (no accepted
    observations) while the camera actively sees the new position. An
    established entry is retired when BOTH hold:

    * it has not accepted an observation for stale_conflict_sec (starved —
      a real cone in view keeps receiving observations), and
    * a same-colour established entry within conflict_radius is fresh
      (accepted an observation within stale_conflict_sec).

    The freshness of the twin is what separates a moved cone from a real
    neighbour: a neighbouring cone is observed whenever the camera looks
    at the area, so it never goes stale while its neighbourhood is being
    confirmed. Ties to nothing transient: the twin must have passed the
    full tentative lifecycle (C1) before it can retire anything.
    """
    conflict_sq = cfg.conflict_radius * cfg.conflict_radius
    doomed: List[int] = []
    for i, c in enumerate(cones):
        if c.tentative or c.color == CONE_UNKNOWN:
            continue
        if now - c.last_seen <= cfg.stale_conflict_sec:
            continue  # still being observed
        for j, twin in enumerate(cones):
            if j == i or twin.tentative or twin.color != c.color:
                continue
            if now - twin.last_seen > cfg.stale_conflict_sec:
                continue  # both starved: not a move signature
            if (c.x - twin.x) ** 2 + (c.y - twin.y) ** 2 <= conflict_sq:
                doomed.append(i)
                break
    for i in reversed(doomed):
        cones.pop(i)
    return len(doomed)


def evict_stale(
    cones: List[MapCone],
    now: float,
    max_age: float,
    tentative_max_age: Optional[float] = None,
) -> int:
    """Drop entries not seen within their age limit. Returns number evicted.

    Tentative entries evict on the (short) tentative age — transient
    sources must die quickly — established entries on max_age.
    """
    t_age = max_age if tentative_max_age is None else tentative_max_age
    before = len(cones)
    cones[:] = [
        c for c in cones
        if (now - c.last_seen) < (t_age if c.tentative else max_age)
    ]
    return before - len(cones)


def publishable(
    cones: List[MapCone], min_observations: int, max_cones: int = 0
) -> List[MapCone]:
    """Established cones confident enough for the planner, most-observed first.

    Tentative entries are never published regardless of their observation
    count. max_cones > 0 caps the output size (keeps the highest-evidence
    cones).
    """
    confirmed = [
        c for c in cones
        if not c.tentative and c.obs_count >= min_observations
    ]
    confirmed.sort(key=lambda c: c.obs_count, reverse=True)
    if max_cones > 0 and len(confirmed) > max_cones:
        confirmed = confirmed[:max_cones]
    # restore a deterministic spatial order for stable downstream buffers
    confirmed.sort(key=lambda c: (round(c.x, 3), round(c.y, 3)))
    return confirmed


def consolidate(
    cones: List[MapCone],
    merge_radius: float,
    gate_sigma_factor: float,
    gate_max_radius: float,
) -> int:
    """Merge stored entries that ended up closer than merge_radius.

    Per-detection merging only sees one detection at a time: a cone whose
    colour flips (blue<->orange misclassification) or whose depth jumps
    leaves several permanent entries 0.3-0.5 m apart that keep trading
    observations forever. A weighted consolidation pass over the stored
    map collapses them (position by observation count, colour by vote
    sum). Physical cones are never this close (FSAC min spacing > 1 m).

    The innovation gate is respected: a tentative entry sitting OUTSIDE an
    established entry's gate radius is a rejected neighbour, not a split
    of the same cone, and must survive (it is either a moved cone being
    re-confirmed or a transient that will age out).
    """
    merge_sq = merge_radius * merge_radius
    removed = 0
    changed = True
    while changed:
        changed = False
        for i in range(len(cones)):
            for j in range(i + 1, len(cones)):
                a, b = cones[i], cones[j]
                dx = a.x - b.x
                dy = a.y - b.y
                d_sq = dx * dx + dy * dy
                if d_sq > merge_sq:
                    continue
                if a.tentative != b.tentative:
                    # Mixed pair: only merge when the tentative side sits
                    # inside the established entry's gate radius.
                    est = a if not a.tentative else b
                    if d_sq > gate_radius(est, gate_sigma_factor, gate_max_radius) ** 2:
                        continue
                n_a, n_b = a.obs_count, b.obs_count
                total = n_a + n_b
                a.x = (a.x * n_a + b.x * n_b) / total
                a.y = (a.y * n_a + b.y * n_b) / total
                a.obs_count = total
                for col, votes in b.color_votes.items():
                    a.color_votes[col] = a.color_votes.get(col, 0) + votes
                a.last_seen = max(a.last_seen, b.last_seen)
                a.first_seen = min(a.first_seen, b.first_seen)
                a.tentative = a.tentative and b.tentative
                a.confirm_count = max(a.confirm_count, b.confirm_count)
                if a.sim_cone_index == 0:
                    a.sim_cone_index = b.sim_cone_index
                cones.pop(j)
                removed += 1
                changed = True
                break
            if changed:
                break
    return removed
