"""Unit tests for the pure cone merge logic (no ROS imports)."""

from cone_mapping.cone_mapper import (
    CONE_BLUE,
    CONE_UNKNOWN,
    CONE_YELLOW,
    covariance_for,
    evict_stale,
    merge_detection,
    new_cone,
    publishable,
)


class TestMergeDetection:
    def test_new_detection_far_from_map_creates_new(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        merged, idx = merge_detection(cones, 10.0, 10.0, CONE_BLUE, now=1.0, merge_radius=0.5)
        assert merged is False
        assert idx is None

    def test_merge_within_radius_weighted_average(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        cones[0].obs_count = 3  # pretend 3 observations already happened at (0,0)
        merged, idx = merge_detection(cones, 0.4, 0.0, CONE_BLUE, now=1.0, merge_radius=0.5)
        assert merged is True and idx == 0
        # weighted: (0*3 + 0.4)/4 = 0.1
        assert abs(cones[0].x - 0.1) < 1e-9
        assert cones[0].obs_count == 4

    def test_merge_nearest_wins(self):
        cones = [
            new_cone(0.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(1.0, 0.0, CONE_YELLOW, now=0.0),
        ]
        merged, idx = merge_detection(cones, 0.9, 0.0, CONE_BLUE, now=1.0, merge_radius=0.5)
        assert merged is True and idx == 1

    def test_boundary_distance_not_merged(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        # exactly at radius -> strictly less-than required, no merge
        merged, _ = merge_detection(cones, 0.5, 0.0, CONE_BLUE, now=1.0, merge_radius=0.5)
        assert merged is False

    def test_color_vote_majority(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        for _ in range(3):
            merge_detection(cones, 0.1, 0.0, CONE_YELLOW, now=1.0, merge_radius=0.5)
        assert cones[0].color == CONE_YELLOW

    def test_unknown_color_does_not_vote(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        merge_detection(cones, 0.1, 0.0, CONE_UNKNOWN, now=1.0, merge_radius=0.5)
        assert cones[0].color == CONE_BLUE
        assert CONE_UNKNOWN not in cones[0].color_votes

    def test_last_seen_refreshed(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        merge_detection(cones, 0.1, 0.0, CONE_BLUE, now=42.0, merge_radius=0.5)
        assert cones[0].last_seen == 42.0


class TestEvictStale:
    def test_old_cone_evicted(self):
        cones = [
            new_cone(0.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(1.0, 0.0, CONE_BLUE, now=100.0),
        ]
        n = evict_stale(cones, now=200.0, max_age=120.0)
        assert n == 1
        assert len(cones) == 1
        assert cones[0].x == 1.0


class TestPublishable:
    def test_min_observations_gate(self):
        cones = [
            new_cone(0.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(1.0, 0.0, CONE_BLUE, now=0.0),
        ]
        cones[1].obs_count = 5
        out = publishable(cones, min_observations=3)
        assert len(out) == 1 and out[0].x == 1.0

    def test_max_cones_cap_keeps_most_observed(self):
        cones = [new_cone(float(i), 0.0, CONE_BLUE, now=0.0) for i in range(10)]
        cones[7].obs_count = 99
        out = publishable(cones, min_observations=1, max_cones=3)
        assert len(out) == 3
        assert any(c.obs_count == 99 for c in out)

    def test_spatial_order_deterministic(self):
        cones = [
            new_cone(5.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(1.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(3.0, 0.0, CONE_BLUE, now=0.0),
        ]
        out = publishable(cones, min_observations=1)
        xs = [c.x for c in out]
        assert xs == sorted(xs)


class TestCovariance:
    def test_shrinks_with_observations(self):
        assert covariance_for(10)[0] < covariance_for(1)[0]
        assert covariance_for(0)[0] == covariance_for(1)[0]
