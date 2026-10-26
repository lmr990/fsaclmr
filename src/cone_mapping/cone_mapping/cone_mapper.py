"""Pure merge logic for the cone mapper (no ROS imports, unit-testable).

Detections arrive in a global map frame (e.g. "track") with an absolute
pose source (GNSS/INS), so mapping reduces to accumulation with known
poses: match each detection to the nearest existing map cone within
merge_radius and update it with an observation-count weighted average,
otherwise create a new entry.
"""

from __future__ import annotations

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

    @property
    def color(self) -> int:
        """Majority colour vote; UNKNOWN when no coloured observation yet."""
        if not self.color_votes:
            return CONE_UNKNOWN
        return max(self.color_votes.items(), key=lambda kv: kv[1])[0]


def merge_detection(
    cones: List[MapCone],
    det_x: float,
    det_y: float,
    det_color: int,
    now: float,
    merge_radius: float,
    det_sim_index: int = 0,
) -> Tuple[bool, Optional[int]]:
    """Fold one detection into the map.

    Returns (merged, index). When merged is True the existing entry at
    `index` was updated (weighted position, observation count, colour
    vote); when False the caller should append a new MapCone.
    """
    merge_radius_sq = merge_radius * merge_radius
    best_idx = -1
    best_dist_sq = merge_radius_sq
    for i, c in enumerate(cones):
        dx = c.x - det_x
        dy = c.y - det_y
        d_sq = dx * dx + dy * dy
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            best_idx = i

    if best_idx < 0:
        return False, None

    c = cones[best_idx]
    # weighted average: old position keeps weight n/(n+1), new detection 1/(n+1)
    n = c.obs_count
    c.x = (c.x * n + det_x) / (n + 1)
    c.y = (c.y * n + det_y) / (n + 1)
    c.obs_count = n + 1
    c.last_seen = now
    if det_sim_index != 0:
        c.sim_cone_index = det_sim_index
    if det_color != CONE_UNKNOWN:
        c.color_votes[det_color] = c.color_votes.get(det_color, 0) + 1
    return True, best_idx


def new_cone(
    det_x: float,
    det_y: float,
    det_color: int,
    now: float,
    det_sim_index: int = 0,
) -> MapCone:
    votes: Dict[int, int] = {}
    if det_color != CONE_UNKNOWN:
        votes[det_color] = 1
    return MapCone(
        x=det_x, y=det_y, obs_count=1, color_votes=votes,
        last_seen=now, sim_cone_index=det_sim_index,
    )


def evict_stale(cones: List[MapCone], now: float, max_age: float) -> int:
    """Drop cones not seen within max_age seconds. Returns number evicted."""
    before = len(cones)
    cones[:] = [c for c in cones if (now - c.last_seen) < max_age]
    return before - len(cones)


def publishable(cones: List[MapCone], min_observations: int, max_cones: int = 0) -> List[MapCone]:
    """Cones confident enough for the planner, most-observed first.

    max_cones > 0 caps the output size (keeps the highest-evidence cones).
    """
    confirmed = [c for c in cones if c.obs_count >= min_observations]
    confirmed.sort(key=lambda c: c.obs_count, reverse=True)
    if max_cones > 0 and len(confirmed) > max_cones:
        confirmed = confirmed[:max_cones]
    # restore a deterministic spatial order for stable downstream buffers
    confirmed.sort(key=lambda c: (round(c.x, 3), round(c.y, 3)))
    return confirmed


def covariance_for(obs_count: int) -> List[float]:
    """Diagonal covariance shrinking with observation count (x, y order)."""
    var = max(0.01, 0.25 / max(1, obs_count))
    return [var, 0.0, 0.0, var]
