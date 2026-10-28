"""Assign stable IDs to the cone map for urinay.

Subscribes the merged cone map (ConeDetectionStamped from cone_mapper) and
publishes a ConeWithIdArray for urinay. Urinay's Delaunay/tree-search relies on
each physical cone keeping the same id across updates, so detections are
matched against the previous map with a globally greedy nearest-neighbour
assignment: matched cones keep their id (position is refreshed), unmatched
detections get the next free id.
"""

import time

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithId, ConeWithIdArray
from driverless_common.common import QOS_LATEST
from rclpy.node import Node

_ASSOC_DIST = 1.0  # m; max displacement of a mapped cone between updates


def match_cones(prev_positions: dict, detections: list, assoc_dist: float) -> tuple:
    """Global-greedy nearest-neighbour association between the previous map
    and the new detection set.

    prev_positions: {id: (x, y)} of the last published map.
    detections: list of (x, y).
    Returns (assignments, next_free_id) where assignments is a list of
    (id, x, y) — matched cones keep their id and take the new position,
    unmatched ones get fresh ids.

    Pairs are consumed in increasing distance instead of detection order, so
    two cones swapping nearest neighbours cannot steal each other's id.
    """
    max_d_sq = assoc_dist * assoc_dist
    candidates = []
    for det_idx, (x, y) in enumerate(detections):
        for cid, (px, py) in prev_positions.items():
            d_sq = (px - x) ** 2 + (py - y) ** 2
            if d_sq < max_d_sq:
                candidates.append((d_sq, det_idx, cid))
    candidates.sort(key=lambda item: item[0])

    det_to_id = {}
    used_prev = set()
    for _d_sq, det_idx, cid in candidates:
        if det_idx in det_to_id or cid in used_prev:
            continue
        det_to_id[det_idx] = cid
        used_prev.add(cid)

    next_free_id = (max(prev_positions) + 1) if prev_positions else 0
    assignments = []
    for det_idx, (x, y) in enumerate(detections):
        if det_idx in det_to_id:
            assignments.append((det_to_id[det_idx], x, y))
        else:
            assignments.append((next_free_id, x, y))
            next_free_id += 1

    return assignments, next_free_id


def dedupe_detections(detections: list, radius: float) -> list:
    """Collapse detections that land on the same physical cone.

    The upstream mapper can emit several entries per cone when detections are
    noisy (seen here as clusters of 5-8 points within half a metre). Urinay
    triangulates those into degenerate triangles with sub-metre edges, which
    corrupts the search, so each cluster is reduced to its centroid.
    """
    if radius <= 0.0:
        return list(detections)
    radius_sq = radius * radius
    clusters = []  # list of [sum_x, sum_y, count]
    for x, y in detections:
        for c in clusters:
            cx, cy = c[0] / c[2], c[1] / c[2]
            if (cx - x) ** 2 + (cy - y) ** 2 < radius_sq:
                c[0] += x
                c[1] += y
                c[2] += 1
                break
        else:
            clusters.append([x, y, 1])
    return [(c[0] / c[2], c[1] / c[2]) for c in clusters]


class ConeIdMapper:
    """Mixin for the ROS node; holds the id state and the conversion."""

    def init_id_mapper(self, node: Node, input_topic: str, output_topic: str, assoc_dist: float,
                       dedupe_radius: float = 1.0, id_ttl_sec: float = 5.0):
        self._assoc_dist = assoc_dist
        self._dedupe_radius = dedupe_radius
        self._id_ttl = id_ttl_sec
        self._prev_positions = {}  # id -> (x, y) of the last published map
        self._prev_last_seen = {}  # id -> monotonic time of last match
        node.create_subscription(ConeDetectionStamped, input_topic, self._cone_map_callback, QOS_LATEST)
        self._id_map_pub = node.create_publisher(ConeWithIdArray, output_topic, QOS_LATEST)

    def _cone_map_callback(self, msg: ConeDetectionStamped):
        detections = dedupe_detections(
            [(c.location.x, c.location.y) for c in msg.cones], self._dedupe_radius
        )
        if not detections:
            return

        now = time.monotonic()
        # Retire ids of cones that have not been seen for a while. Otherwise
        # they would stay candidates forever and could capture a new cone.
        stale_ids = [
            cid for cid, seen in self._prev_last_seen.items()
            if now - seen > self._id_ttl
        ]
        for cid in stale_ids:
            self._prev_positions.pop(cid, None)
            self._prev_last_seen.pop(cid, None)

        assignments, _ = match_cones(self._prev_positions, detections, self._assoc_dist)

        out = ConeWithIdArray()
        out.header = msg.header
        for cid, x, y in assignments:
            cone = ConeWithId()
            cone.id = cid
            cone.position.x = float(x)
            cone.position.y = float(y)
            cone.position.z = 0.0
            out.cones.append(cone)

        current_positions = {cid: (x, y) for cid, x, y in assignments}
        # Keep ids that were not matched in this message alive until the TTL,
        # otherwise a cone missed for a single update would immediately lose
        # its id and be re-created as a new cone.
        self._prev_positions.update(current_positions)
        self._prev_last_seen.update({cid: now for cid in current_positions})
        self._id_map_pub.publish(out)
