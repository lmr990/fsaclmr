"""Unit tests for the pure cone merge logic (no ROS imports).

Covers the static-world rules: tentative lifecycle (C1), innovation gate
(C2), conflict retirement (C3), consolidation and eviction.

Checklist mapping (docs/occlusion_ghost_acceptance.md):
  TestProcessDetectionLifecycle          -> C1
  TestInnovationGate.test_C2_*           -> C2
  TestConflictRetirement.test_C3_*       -> C3
  TestConsolidate / TestEvictStale / ... -> C1/C3 supporting behavior
"""

from cone_mapping.cone_mapper import (
    CONE_BLUE,
    CONE_UNKNOWN,
    CONE_YELLOW,
    MapCone,
    MapperConfig,
    consolidate,
    covariance_for,
    evict_stale,
    gate_radius,
    new_cone,
    process_detection,
    publishable,
    retire_superseded,
)

CFG = MapperConfig()  # defaults: merge 0.5, confirm 4 obs / 1.0s / 0.15m,
# gate 2.5*sigma cap 0.4m, conflict 1.5m, stale 5s


def establish(cone: MapCone, obs_count: int) -> MapCone:
    """Simulate a long-established entry (position settled at its x, y)."""
    cone.tentative = False
    cone.obs_count = obs_count
    return cone


class TestProcessDetectionLifecycle:
    def test_first_detection_creates_tentative(self):
        cones = []
        disp = process_detection(cones, 2.0, 1.0, CONE_BLUE, now=0.0, cfg=CFG)
        assert disp == "created"
        assert len(cones) == 1
        assert cones[0].tentative is True
        assert cones[0].confirm_count == 1
        assert cones[0].first_seen == 0.0

    def test_C1_confirm_requires_matching_colour(self):
        cones = []
        # creating observation + 3 tight blue observations: confirm_count = 3
        for k in range(1, 4):
            process_detection(cones, 2.0, 1.0, CONE_BLUE, now=0.1 * k, cfg=CFG)
        assert cones[0].confirm_count == 3
        # a tight observation of another colour does NOT count
        process_detection(cones, 2.0, 1.0, CONE_YELLOW, now=0.4, cfg=CFG)
        assert cones[0].confirm_count == 3
        # an UNKNOWN observation carries no colour: does not count either
        process_detection(cones, 2.0, 1.0, CONE_UNKNOWN, now=0.5, cfg=CFG)
        assert cones[0].confirm_count == 3
        # the 4th matching observation, arriving once the 1.0 s span is
        # reached, promotes the entry
        disp = process_detection(cones, 2.0, 1.0, CONE_BLUE, now=1.1, cfg=CFG)
        assert disp == "promoted"
        assert cones[0].tentative is False

    def test_tentative_never_published_even_with_many_observations(self):
        cones = []
        # 10 tight observations in half a second: plenty of evidence but
        # not enough TIME span to distinguish a static cone from a source
        # that happened to dwell briefly.
        for k in range(10):
            process_detection(cones, 2.0, 1.0, CONE_BLUE, now=0.05 * k, cfg=CFG)
        assert cones[0].obs_count == 10
        assert cones[0].tentative is True
        assert publishable(cones, min_observations=3) == []

    def test_promotes_after_span_of_tight_observations(self):
        cones = []
        disps = []
        for k in range(15):
            disp = process_detection(cones, 2.0, 1.0, CONE_BLUE, now=0.1 * k, cfg=CFG)
            disps.append(disp)
        assert "promoted" in disps  # once the 1.0 s span is reached
        assert cones[0].tentative is False
        out = publishable(cones, min_observations=5)
        assert len(out) == 1 and abs(out[0].x - 2.0) < 1e-9

    def test_walking_source_never_promotes(self):
        cones = []
        # A person walking past at 3 m/s: every observation is 0.3 m from
        # the previous, so the entry gets dragged but never sees a tight
        # observation after the first.
        for k in range(20):
            process_detection(cones, 2.0 + 0.3 * k, 1.0, CONE_BLUE, now=0.1 * k, cfg=CFG)
        assert cones[0].tentative is True
        assert cones[0].confirm_count == 1  # only the creating observation was tight
        assert publishable(cones, min_observations=1) == []

    def test_tentative_evicted_quickly(self):
        cones = []
        process_detection(cones, 2.0, 1.0, CONE_BLUE, now=0.0, cfg=CFG)
        evicted = evict_stale(cones, now=6.0, max_age=120.0, tentative_max_age=5.0)
        assert evicted == 1 and cones == []


class TestInnovationGate:
    def test_C2_established_not_dragged_by_bad_stream(self):
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30)]
        # gate at n=30: min(2.5*sqrt(0.25/30), 0.4) = min(0.228, 0.4) = 0.228 m;
        # a stream of bad observations at 0.4 m must not move the entry
        for k in range(10):
            disp = process_detection(cones, 0.4, 0.0, CONE_BLUE, now=1.0 + 0.1 * k, cfg=CFG)
        assert cones[0].x == 0.0 and cones[0].obs_count == 30  # undragged
        # each rejected observation instead starts/feeds a tentative
        assert any(c.tentative for c in cones)

    def test_C2_young_established_not_dragged_by_bad_stream(self):
        # Young entries (obs_count just past the publish gate) have a large
        # sigma and are the main drag victims; the absolute gate cap must
        # protect them. n=5: sigma gate would be 0.559 m (wider than
        # association!), cap holds it at 0.4 m.
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=5)]
        for k in range(10):
            process_detection(cones, 0.45, 0.0, CONE_BLUE, now=1.0 + 0.1 * k, cfg=CFG)
        assert cones[0].x == 0.0 and cones[0].obs_count == 5  # undragged

    def test_C2_young_established_still_merges_normal_observations(self):
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=5)]
        # 0.2 m is normal depth noise: must still merge
        disp = process_detection(cones, 0.2, 0.0, CONE_BLUE, now=1.0, cfg=CFG)
        assert disp == "merged"
        assert cones[0].obs_count == 6
        assert abs(cones[0].x - 0.2 / 6) < 1e-9

    def test_established_entry_accepts_within_gate(self):
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30)]
        disp = process_detection(cones, 0.2, 0.0, CONE_BLUE, now=1.0, cfg=CFG)
        assert disp == "merged"
        assert cones[0].obs_count == 31
        assert abs(cones[0].x - 0.2 / 31) < 1e-9

    def test_persistent_gated_source_confirms_and_supersedes_starved_entry(self):
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30)]
        # The physical cone moved 0.4 m (kicked): from t=10 on, all
        # observations land at the new position; the old entry is starved.
        for k in range(20):
            process_detection(cones, 0.4, 0.0, CONE_BLUE, now=10.0 + 0.1 * k, cfg=CFG)
        # Periodic retirement once the new position is established and the
        # old entry has been starved longer than stale_conflict_sec.
        retired = retire_superseded(cones, now=16.0, cfg=CFG)
        assert retired == 1
        assert len(cones) == 1
        assert cones[0].tentative is False
        assert abs(cones[0].x - 0.4) < 0.05

    def test_transient_gated_source_leaves_established_untouched(self):
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30)]
        # A couple of gated observations (e.g. a partial occluder that
        # slipped through) then nothing: the tentative dies, the
        # established entry survives retirement (no fresh same-colour twin).
        for k in range(2):
            process_detection(cones, 0.4, 0.0, CONE_BLUE, now=10.0 + 0.1 * k, cfg=CFG)
        evict_stale(cones, now=20.0, max_age=120.0, tentative_max_age=5.0)
        retire_superseded(cones, now=20.0, cfg=CFG)
        assert len(cones) == 1
        assert cones[0].tentative is False and cones[0].x == 0.0


class TestConflictRetirement:
    @staticmethod
    def _map_with_neighbour(neighbour_last_seen: float):
        cones = []
        process_detection(cones, 5.0, 0.0, CONE_YELLOW, now=0.0, cfg=CFG)
        establish(cones[0], obs_count=30)
        cones[0].last_seen = neighbour_last_seen
        return cones

    @staticmethod
    def _grow_new_cone(cones):
        """Feed 1.5 s of tight observations at (6, 0) until established."""
        for k in range(15):
            process_detection(cones, 6.0, 0.0, CONE_YELLOW, now=10.0 + 0.1 * k, cfg=CFG)
        assert cones[-1].tentative is False

    def test_C3_retires_stale_same_colour_within_conflict_radius(self):
        cones = self._map_with_neighbour(neighbour_last_seen=0.0)  # 10+ s unseen
        self._grow_new_cone(cones)
        retired = retire_superseded(cones, now=16.0, cfg=CFG)
        assert retired == 1
        assert len(cones) == 1 and abs(cones[0].x - 6.0) < 1e-9

    def test_C3_cone_moved_1m_becomes_only_published_entry(self):
        # Cone established at the origin, observed until t=0, then the
        # physical cone is moved ~1 m away. New observations start at 0.1
        # and are beyond merge_radius, so they build a fresh tentative at
        # the new spot; the old entry is starved and retired by the
        # periodic pass.
        cones = [establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=50)]
        for k in range(1, 20):
            process_detection(cones, 1.0, 0.0, CONE_BLUE, now=0.1 * k, cfg=CFG)
        assert cones[-1].tentative is False
        assert abs(cones[-1].x - 1.0) < 0.05
        # old entry still present right after promotion (not yet starved
        # for longer than stale_conflict_sec relative to a fresh twin)
        assert len(cones) == 2
        retired = retire_superseded(cones, now=6.0, cfg=CFG)
        assert retired == 1
        assert len(cones) == 1
        # the published map now contains only the new position
        out = publishable(cones, min_observations=3)
        assert len(out) == 1 and abs(out[0].x - 1.0) < 0.05

    def test_keeps_fresh_same_colour_established(self):
        cones = self._map_with_neighbour(neighbour_last_seen=14.5)  # actively observed
        self._grow_new_cone(cones)
        retired = retire_superseded(cones, now=16.0, cfg=CFG)
        assert retired == 0
        assert len(cones) == 2  # real neighbouring cone survives

    def test_keeps_stale_different_colour_established(self):
        # new yellow cone confirms 1.0 m from a stale blue entry: different
        # colour, never retired even though the blue entry is stale
        cones = [establish(new_cone(6.0, 0.0, CONE_BLUE, now=0.0), obs_count=30)]
        for k in range(15):
            process_detection(cones, 6.0, 1.0, CONE_YELLOW, now=10.0 + 0.1 * k, cfg=CFG)
        retired = retire_superseded(cones, now=16.0, cfg=CFG)
        assert retired == 0
        assert len(cones) == 2

    def test_keeps_stale_established_beyond_conflict_radius(self):
        cones = [establish(new_cone(8.0, 0.0, CONE_YELLOW, now=0.0), obs_count=30)]
        self._grow_new_cone(cones)
        retired = retire_superseded(cones, now=16.0, cfg=CFG)
        assert retired == 0
        assert len(cones) == 2

    def test_two_starved_entries_do_not_retire_each_other(self):
        cones = [
            establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30),
            establish(new_cone(0.8, 0.0, CONE_BLUE, now=0.0), obs_count=30),
        ]
        retired = retire_superseded(cones, now=100.0, cfg=CFG)
        assert retired == 0 and len(cones) == 2


class TestConsolidate:
    def test_merges_two_tentatives_within_radius(self):
        cones = [
            new_cone(0.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(0.3, 0.0, CONE_BLUE, now=0.0),
        ]
        removed = consolidate(cones, merge_radius=0.5, gate_sigma_factor=2.5,
                              gate_max_radius=0.4)
        assert removed == 1 and len(cones) == 1
        assert cones[0].obs_count == 2 and cones[0].tentative is True

    def test_merges_two_established_within_radius(self):
        cones = [
            establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=10),
            establish(new_cone(0.3, 0.0, CONE_BLUE, now=0.0), obs_count=5),
        ]
        removed = consolidate(cones, merge_radius=0.5, gate_sigma_factor=2.5,
                              gate_max_radius=0.4)
        assert removed == 1 and len(cones) == 1
        assert cones[0].tentative is False and cones[0].obs_count == 15

    def test_keeps_tentative_outside_established_gate(self):
        cones = [
            establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=30),
            new_cone(0.4, 0.0, CONE_BLUE, now=1.0),  # gate at n=30 is 0.228 m
        ]
        removed = consolidate(cones, merge_radius=0.5, gate_sigma_factor=2.5,
                              gate_max_radius=0.4)
        assert removed == 0 and len(cones) == 2

    def test_merges_tentative_inside_established_gate(self):
        cones = [
            establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=9),
            new_cone(0.3, 0.0, CONE_BLUE, now=1.0),  # gate at n=9 is 0.4 m (cap)
        ]
        removed = consolidate(cones, merge_radius=0.5, gate_sigma_factor=2.5,
                              gate_max_radius=0.4)
        assert removed == 1 and len(cones) == 1
        assert cones[0].tentative is False and cones[0].obs_count == 10


class TestGateRadius:
    def test_C2_cap_bounds_young_entries_sigma_floors_mature_ones(self):
        young = establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=5)
        mid = establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=16)
        old = establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=25)
        assert gate_radius(young, 2.5, 0.4) == 0.4  # capped (sigma gate 0.559)
        assert abs(gate_radius(mid, 2.5, 0.4) - 0.3125) < 1e-9  # below cap
        assert gate_radius(old, 2.5, 0.4) == 0.25  # sigma floor 0.1
        assert gate_radius(old, 2.5, 0.4) < gate_radius(mid, 2.5, 0.4) < gate_radius(young, 2.5, 0.4)


class TestEvictStale:
    def test_tentative_evicts_on_short_age_established_on_max_age(self):
        tent = new_cone(0.0, 0.0, CONE_BLUE, now=0.0)
        est = establish(new_cone(1.0, 0.0, CONE_BLUE, now=0.0), obs_count=10)
        cones = [tent, est]
        evicted = evict_stale(cones, now=6.0, max_age=120.0, tentative_max_age=5.0)
        assert evicted == 1 and len(cones) == 1 and cones[0] is est

    def test_without_tentative_age_uses_max_age_for_all(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        evicted = evict_stale(cones, now=100.0, max_age=120.0)
        assert evicted == 0
        evicted = evict_stale(cones, now=200.0, max_age=120.0)
        assert evicted == 1 and cones == []

    def test_old_cone_evicted(self):
        cones = [
            new_cone(0.0, 0.0, CONE_BLUE, now=0.0),
            new_cone(1.0, 0.0, CONE_BLUE, now=100.0),
        ]
        cones[1].tentative = False
        n = evict_stale(cones, now=200.0, max_age=120.0)
        assert n == 1
        assert len(cones) == 1
        assert cones[0].x == 1.0


class TestPublishable:
    def test_min_observations_gate(self):
        cones = [
            establish(new_cone(0.0, 0.0, CONE_BLUE, now=0.0), obs_count=1),
            establish(new_cone(1.0, 0.0, CONE_BLUE, now=0.0), obs_count=5),
        ]
        out = publishable(cones, min_observations=3)
        assert len(out) == 1 and out[0].x == 1.0

    def test_tentative_entries_never_published(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        cones[0].obs_count = 99
        assert publishable(cones, min_observations=1) == []

    def test_max_cones_cap_keeps_most_observed(self):
        cones = [new_cone(float(i), 0.0, CONE_BLUE, now=0.0) for i in range(10)]
        for c in cones:
            c.tentative = False
        cones[7].obs_count = 99
        out = publishable(cones, min_observations=1, max_cones=3)
        assert len(out) == 3
        assert any(c.obs_count == 99 for c in out)

    def test_spatial_order_deterministic(self):
        cones = [
            establish(new_cone(5.0, 0.0, CONE_BLUE, now=0.0), obs_count=1),
            establish(new_cone(1.0, 0.0, CONE_BLUE, now=0.0), obs_count=1),
            establish(new_cone(3.0, 0.0, CONE_BLUE, now=0.0), obs_count=1),
        ]
        out = publishable(cones, min_observations=1)
        xs = [c.x for c in out]
        assert xs == sorted(xs)


class TestColourVotes:
    def test_colour_vote_majority_survives_lifecycle(self):
        cones = []
        for k in range(15):
            process_detection(cones, 2.0, 1.0, CONE_YELLOW, now=0.1 * k, cfg=CFG)
        assert cones[0].color == CONE_YELLOW
        assert cones[0].tentative is False

    def test_unknown_color_does_not_vote(self):
        cones = [new_cone(0.0, 0.0, CONE_BLUE, now=0.0)]
        process_detection(cones, 0.1, 0.0, CONE_UNKNOWN, now=1.0, cfg=CFG)
        assert cones[0].color == CONE_BLUE
        assert CONE_UNKNOWN not in cones[0].color_votes


class TestCovariance:
    def test_shrinks_with_observations(self):
        assert covariance_for(10)[0] < covariance_for(1)[0]
        assert covariance_for(0)[0] == covariance_for(1)[0]
