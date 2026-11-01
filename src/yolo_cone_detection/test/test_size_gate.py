"""Unit tests for the depth-size consistency gate (checklist item B).

fy ≈ 700 px is a ZED 2i RGB focal length at 720p; expected pixel heights
below use h_px = fy * H / z.

Checklist mapping:
  test_B1_normal_cone_kept                  -> B-④① 正常锥保留
  test_B2_partial_occlusion_short_dropped   -> B-④② 框太矮（部分遮挡）丢弃
  test_B3_person_scale_tall_dropped         -> B-④③ 框太高（人/马甲）丢弃
  test_B4_edge_clipped_box_kept             -> B-④④ 贴边框保留（矮框豁免）
  test_B5_orange_band_admits_large_cone     -> B 锥高按类别（大橙锥放行）
  test_B6_blueyellow_tighter_tall_bound     -> B 锥高按类别（蓝黄上界更紧）
  test_B7_edge_clipped_tall_still_dropped   -> B 豁免只豁矮不豁高
"""

from yolo_cone_detection.size_gate import size_gate_reject

FY = 700.0
H_SMALL = 0.305
H_LARGE = 0.505
MIN_RATIO = 0.55
MAX_RATIO = 2.0


def gate(bbox_h, z, h_small=H_SMALL, h_large=H_SMALL, touches_edge=False):
    return size_gate_reject(
        bbox_h, z, FY, h_small, h_large, MIN_RATIO, MAX_RATIO, touches_edge
    )


class TestL1SizeGate:
    def test_B1_normal_cone_kept(self):
        # small cone at 3 m: expected height = 700*0.305/3 = 71 px
        assert gate(71.0, z=3.0) is False
        # box slightly loose (base flare / shadow): ratio 1.3
        assert gate(92.0, z=3.0) is False
        # far cone at 8 m: expected 27 px, measured 30 px
        assert gate(30.0, z=8.0) is False

    def test_B2_partial_occlusion_short_dropped(self):
        # cone at 3 m occluded by a hand at 0.7 m: depth window lands on
        # the hand (z=0.7 -> expected 305 px) while only the cone's upper
        # half is visible in the box (60 px): ratio ~0.2 -> reject
        assert gate(60.0, z=0.7) is True
        # cone at 5 m with only its top third visible (box 20 px vs
        # expected 43 px): ratio 0.47 -> reject
        assert gate(20.0, z=5.0) is True

    def test_B3_person_scale_tall_dropped(self):
        # standing person (1.7 m) at 3 m detected as a cone: box 397 px,
        # expected 71 px -> ratio 5.6 -> reject (marshal in orange vest)
        assert gate(397.0, z=3.0, h_small=H_SMALL, h_large=H_LARGE) is True
        # same person at 5 m: box 238 px vs expected 43 px
        assert gate(238.0, z=5.0, h_small=H_SMALL, h_large=H_LARGE) is True

    def test_B4_edge_clipped_box_kept(self):
        # cone at 3 m whose box is clipped by the frame bottom: measured
        # height 40 px is below min_ratio*71=39... use 30 px (clearly
        # below the band) -> the too-short check is skipped for edge boxes
        assert gate(30.0, z=3.0, touches_edge=True) is False

    def test_B5_orange_band_admits_large_cone(self):
        # large orange cone (0.505 m) at 3 m: box 118 px = 1.66x the small
        # height. Single shared tolerance band: it stays inside the blue
        # band too (documented slack); the point is orange MUST keep it.
        assert gate(118.0, z=3.0, h_small=H_SMALL, h_large=H_LARGE) is False

    def test_B6_blueyellow_tighter_tall_bound(self):
        # 0.7 m object at 3 m (160 px): too tall for blue/yellow (band cap
        # 2.0*71 = 142 px) but inside orange's large-cone band (cap 236 px)
        assert gate(160.0, z=3.0, h_small=H_SMALL, h_large=H_SMALL) is True
        assert gate(160.0, z=3.0, h_small=H_SMALL, h_large=H_LARGE) is False

    def test_B7_edge_clipped_tall_still_dropped(self):
        # edge exemption only forgives SHORT boxes: a clipped box that is
        # still too tall is impossible regardless of clipping
        assert gate(400.0, z=3.0, h_small=H_SMALL, h_large=H_LARGE,
                    touches_edge=True) is True
