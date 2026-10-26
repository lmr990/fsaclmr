"""Standalone reproduction of ft_planner's path calculation on the synthetic
corridor, to debug why calculate_path_in_global_frame yields no boundaries."""

import numpy as np

from fsd_path_planning import MissionTypes, PathPlanner
from fsd_path_planning.utils.math_utils import unit_2d_vector_from_angle
from driverless_msgs.msg import Cone

TRACK_HALF_WIDTH = 1.5
STEP, LENGTH, CURVE = 3.0, 30.0, 0.15


def corridor(side):
    pts = []
    for i in range(int(LENGTH / STEP) + 1):
        x = i * STEP
        y_mid = CURVE * x * x / 10.0
        pts.append([x, y_mid + TRACK_HALF_WIDTH * side])
    return np.array(pts)


planner = PathPlanner(MissionTypes.trackdrive)

yellow = corridor(+1)   # REP-103: +y left ... FSAC: yellow on RIGHT
blue = corridor(-1)
empty = np.array([])
car_position = np.array([0.0, 0.0])
car_direction = unit_2d_vector_from_angle(0.0)

for label, cones in [
    ("yellow=+y, blue=-y (当前demo约定)", [empty, yellow, blue, empty, empty]),
    ("yellow=-y, blue=+y (颜色对调)", [empty, corridor(-1), corridor(+1), empty, empty]),
]:
    try:
        result = planner.calculate_path_in_global_frame(
            cones, car_position, car_direction, return_intermediate_results=True
        )
        path, ob, oy, vb, vy, _, _ = result
        print(f"[{label}] path={0 if path is None else len(path)}点 "
              f"ordered_blues={len(ob)} ordered_yellows={len(oy)} "
              f"virt_blues={len(vb)} virt_yellows={len(vy)}")
        if path is not None and len(path):
            print("   path 前3:", np.round(path[:3], 2).tolist())
    except Exception as e:
        print(f"[{label}] 异常: {type(e).__name__}: {e}")
