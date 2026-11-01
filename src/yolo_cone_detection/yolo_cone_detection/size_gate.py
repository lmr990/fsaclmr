"""Pure depth-size consistency gate for cone detections (no ROS imports).

A real cone's apparent pixel height must match its sampled distance:

    h_px ≈ fy * H / z

so the ratio between the measured box height and the expected height at z
is roughly the apparent height in cone-height units and is DISTANCE
INDEPENDENT. Two rejection directions, occluder-agnostic:

* too SHORT — only part of the cone is visible: a hand or a person walking
  past occludes the cone, the depth window near the box base lands on the
  occluder (closer than the cone), and the back-projected "cone" would be
  placed at the occluder's range. This is the classic ghost-cone source.
* too TALL — the box cannot be a cone at that distance: e.g. a marshal in
  an orange vest detected as an orange cone, or a person's torso.

Cone heights follow FSAC/FSG rules: small cones (blue/yellow/red-small)
0.305 m, large orange cones 0.505 m. The image cannot distinguish small
from large orange cones, so orange-class boxes are checked against the
whole [small, large] band: too-short is judged against the SMALL height,
too-tall against the LARGE height. Blue/yellow boxes are judged against
the small height on both sides (no large variant exists in those colours).

Boxes clipped by the top/bottom image edge lose apparent height, so the
too-short half is skipped for them (the too-tall half still applies:
clipping only shrinks the box, so a clipped box exceeding the tall bound
is still impossible).

The tolerance band (min_ratio / max_ratio) is a PRIOR, not calibrated:
see docs/occlusion_ghost_acceptance.md for the calibration plan.
"""

from __future__ import annotations


def size_gate_reject(
    bbox_h: float,
    z: float,
    fy: float,
    h_small: float,
    h_large: float,
    min_ratio: float,
    max_ratio: float,
    touches_edge: bool,
) -> bool:
    """Return True when a box's apparent height contradicts its depth.

    bbox_h      — apparent pixel height of the detection box
    z           — sampled depth for this box [m] (already range-gated)
    fy          — vertical focal length [px]
    h_small     — smallest plausible cone height for this class [m]
    h_large     — largest plausible cone height for this class [m]
    min_ratio   — reject when bbox_h < min_ratio * expected(h_small)
    max_ratio   — reject when bbox_h > max_ratio * expected(h_large)
    touches_edge— box clipped by the top/bottom image edge
    """
    expected_min = fy * h_small / z
    expected_max = fy * h_large / z
    if bbox_h > max_ratio * expected_max:
        return True
    if not touches_edge and bbox_h < min_ratio * expected_min:
        return True
    return False
