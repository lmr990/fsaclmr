"""Assign stable IDs to the cone map for urinay.

Subscribes the merged cone map (ConeDetectionStamped from cone_mapper) and
publishes a ConeWithIdArray for urinay. Urinay's Delaunay/tree-search relies on
each physical cone keeping the same id across updates, so detections are
nearest-neighbour matched against the previous map: matched cones keep their
id (and position), unmatched detections get the next free id.
"""

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithId, ConeWithIdArray
from driverless_common.common import QOS_LATEST
from rclpy.node import Node

_ASSOC_DIST = 1.0  # m; max displacement of a mapped cone between updates


def match_cones(prev_positions: dict, detections: list, assoc_dist: float) -> tuple:
    """Nearest-neighbour data association between the previous map and the
    new detection set.

    prev_positions: {id: (x, y)} of the last published map.
    detections: list of (x, y).
    Returns (assignments, next_free_id) where assignments is a list of
    (detection_index, id, x, y) — matched cones keep their previous id and
    previous position, unmatched ones get fresh ids.
    """
    assignments = []
    used_prev = set()
    next_free_id = (max(prev_positions) + 1) if prev_positions else 0

    for x, y in detections:
        best_id = None
        best_dist_sq = assoc_dist * assoc_dist
        for cid, (px, py) in prev_positions.items():
            if cid in used_prev:
                continue
            d_sq = (px - x) ** 2 + (py - y) ** 2
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_id = cid
        if best_id is not None:
            used_prev.add(best_id)
            assignments.append((best_id, x, y))
        else:
            assignments.append((next_free_id, x, y))
            next_free_id += 1

    return assignments, next_free_id


class ConeIdMapper:
    """Mixin for the ROS node; holds the id state and the conversion."""

    def init_id_mapper(self, node: Node, input_topic: str, output_topic: str, assoc_dist: float):
        self._assoc_dist = assoc_dist
        self._prev_positions = {}  # id -> (x, y) of the last published map
        node.create_subscription(ConeDetectionStamped, input_topic, self._cone_map_callback, QOS_LATEST)
        self._id_map_pub = node.create_publisher(ConeWithIdArray, output_topic, QOS_LATEST)

    def _cone_map_callback(self, msg: ConeDetectionStamped):
        detections = [(c.location.x, c.location.y) for c in msg.cones]
        if not detections:
            return

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

        self._prev_positions = {cid: (x, y) for cid, x, y in assignments}
        self._id_map_pub.publish(out)
