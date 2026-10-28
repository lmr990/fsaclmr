"""Unit tests for cone id association (pure helpers, no ROS spinning)."""

import numpy as np

from planners.cone_id_mapper import dedupe_detections, match_cones


class TestMatchCones:
    def test_empty_previous_map_assigns_sequential_ids(self):
        assignments, next_id = match_cones({}, [(0.0, 0.0), (5.0, 0.0)], 1.0)
        assert assignments == [(0, 0.0, 0.0), (1, 5.0, 0.0)]
        assert next_id == 2

    def test_ids_survive_detection_reordering(self):
        prev = {3: (0.0, 0.0), 7: (5.0, 0.0)}
        assignments, _ = match_cones(prev, [(5.1, 0.0), (0.1, 0.0)], 1.0)
        by_id = {cid: (x, y) for cid, x, y in assignments}
        assert by_id[7] == (5.1, 0.0)
        assert by_id[3] == (0.1, 0.0)

    def test_matched_cone_takes_new_position(self):
        assignments, _ = match_cones({0: (0.0, 0.0)}, [(0.2, 0.1)], 1.0)
        assert assignments == [(0, 0.2, 0.1)]

    def test_unmatched_cone_gets_next_free_id(self):
        assignments, next_id = match_cones({0: (0.0, 0.0)}, [(3.0, 0.0)], 1.0)
        assert assignments == [(1, 3.0, 0.0)]
        assert next_id == 2

    def test_far_detections_are_not_matched(self):
        assignments, _ = match_cones({4: (0.0, 0.0)}, [(2.0, 0.0)], 1.0)
        assert assignments[0][0] == 5


class TestDedupeDetections:
    def test_cluster_is_collapsed_to_centroid(self):
        points = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0)]
        deduped = dedupe_detections(points, 1.0)
        assert len(deduped) == 1
        assert np.allclose(deduped[0], (0.2, 0.0))

    def test_separate_cones_are_kept(self):
        points = [(0.0, 0.0), (3.0, 0.0)]
        assert dedupe_detections(points, 1.0) == points

    def test_zero_radius_is_passthrough(self):
        points = [(0.0, 0.0), (0.0, 0.0)]
        assert dedupe_detections(points, 0.0) == points
