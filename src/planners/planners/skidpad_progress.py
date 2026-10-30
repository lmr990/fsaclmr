"""Monotonic progress tracking for the skidpad window publisher.

Pure Python/numpy; no ROS imports. The tracker uses a global nearest point,
which is ambiguous at the figure-eight intersection, so the node only ever
publishes a short window ahead of this progress value. The progress itself
is searched in a local arc-length window and never jumps backwards.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from planners.skidpad_geometry import SEGMENT_LAP, SkidpadPath

STATE_WAITING = "WAITING_FIT"
STATE_DRIVING = "DRIVING"
STATE_FINISHED = "FINISHED"
STATE_FAULT = "FAULT"
STATE_SHUTDOWN = "SHUTDOWN"


@dataclass
class ProgressConfig:
    search_back: float = 1.0       # [m]
    search_ahead: float = 8.0      # [m]
    window_len: float = 18.0       # [m] published slice, > tracker max lookahead
    off_path_dist: float = 2.0     # [m]
    off_path_time: float = 1.0     # [s]
    # The unchanged tracker stops when its minimum lookahead point (2.0 m)
    # reaches the end at zero target speed, i.e. ~2 m before the geometric
    # end. The finish threshold must be at least that lookahead.
    finish_margin: float = 3.0     # [m]
    max_backtrack: float = 0.1     # [m] tolerance for nearest-point noise


@dataclass
class ProgressUpdate:
    s_prog: float
    segment: str
    lap: int
    state: str
    nearest_dist: float
    window: Optional[Tuple[int, int]]  # inclusive index range, None = do not publish


class SkidpadProgress:
    def __init__(self, path: SkidpadPath, config: Optional[ProgressConfig] = None):
        self.path = path
        self.config = config or ProgressConfig()
        self.s_prog = 0.0
        self.initialized = False
        self.state = STATE_WAITING
        self._off_path_since: Optional[float] = None
        self._last_nearest = float("inf")
        self._entry_indices = np.flatnonzero(
            np.asarray(path.segments, dtype=object) == "ENTRY"
        )

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self.s_prog = 0.0
        self.initialized = False
        self.state = STATE_WAITING
        self._off_path_since = None
        self._last_nearest = float("inf")

    def shutdown(self) -> None:
        self.state = STATE_SHUTDOWN

    # ------------------------------------------------------------------ #
    def _nearest_in_range(self, x: float, y: float, lo: float, hi: float) -> Tuple[int, float]:
        arc = self.path.arc_length
        mask = (arc >= lo) & (arc <= hi)
        if not np.any(mask):
            return -1, float("inf")
        idx = np.flatnonzero(mask)
        d = np.linalg.norm(
            self.path.points[idx] - np.array([x, y], dtype=float)[None, :], axis=1
        )
        best = int(idx[int(np.argmin(d))])
        return best, float(np.min(d))

    def initialize(self, x: float, y: float) -> bool:
        """Locate the car on the ENTRY straight (the only valid start region)."""
        if len(self._entry_indices) == 0:
            return False
        p = np.array([x, y], dtype=float)
        d = np.linalg.norm(self.path.points[self._entry_indices] - p[None, :], axis=1)
        best = int(self._entry_indices[int(np.argmin(d))])
        self.s_prog = float(self.path.arc_length[best])
        self.initialized = True
        self.state = STATE_DRIVING
        return True

    def window_indices(self, s: Optional[float] = None) -> Optional[Tuple[int, int]]:
        if s is None:
            s = self.s_prog
        arc = self.path.arc_length
        if s >= arc[-1] - self.config.finish_margin:
            return None
        i0 = int(np.searchsorted(arc, s, side="left"))
        i1 = int(np.searchsorted(arc, s + self.config.window_len, side="right")) - 1
        i0 = min(max(i0, 0), len(arc) - 1)
        i1 = min(max(i1, i0), len(arc) - 1)
        if i1 - i0 < 1:
            return None
        return i0, i1

    def update(
        self, x: float, y: float, now: float
    ) -> ProgressUpdate:
        """Advance progress to the nearest point in the local search window."""
        if self.state in (STATE_FAULT, STATE_FINISHED, STATE_SHUTDOWN):
            return ProgressUpdate(
                self.s_prog,
                self.path.segment_at(self.s_prog),
                SEGMENT_LAP.get(self.path.segment_at(self.s_prog), 0),
                self.state,
                self._last_nearest,
                None,
            )

        if not self.initialized:
            if not self.initialize(x, y):
                return ProgressUpdate(
                    0.0, "ENTRY", 0, STATE_WAITING, float("inf"), None
                )

        cfg = self.config
        idx, dist = self._nearest_in_range(
            x, y, self.s_prog - cfg.search_back, self.s_prog + cfg.search_ahead
        )
        self._last_nearest = dist
        if idx >= 0:
            candidate = float(self.path.arc_length[idx])
            # Strict monotonic progress: the local nearest point may sit a
            # few centimetres behind due to noise, but progress never moves
            # backwards (the intersection would otherwise jump segments).
            self.s_prog = max(self.s_prog, candidate)

        if dist > cfg.off_path_dist:
            if self._off_path_since is None:
                self._off_path_since = now
            elif now - self._off_path_since >= cfg.off_path_time:
                self.state = STATE_FAULT
                return ProgressUpdate(
                    self.s_prog,
                    self.path.segment_at(self.s_prog),
                    SEGMENT_LAP.get(self.path.segment_at(self.s_prog), 0),
                    STATE_FAULT,
                    dist,
                    None,
                )
        else:
            self._off_path_since = None

        if self.s_prog >= self.path.arc_length[-1] - cfg.finish_margin:
            self.state = STATE_FINISHED
            return ProgressUpdate(
                self.s_prog,
                "EXIT",
                SEGMENT_LAP["EXIT"],
                STATE_FINISHED,
                dist,
                None,
            )

        seg = self.path.segment_at(self.s_prog)
        return ProgressUpdate(
            self.s_prog,
            seg,
            SEGMENT_LAP.get(seg, 0),
            self.state,
            dist,
            self.window_indices(self.s_prog),
        )
