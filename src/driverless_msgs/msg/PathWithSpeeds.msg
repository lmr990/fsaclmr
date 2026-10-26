# Path with per-point target speed and curvature, produced by the planner
# and consumed by the path tracker.
#
# header.stamp keeps the source (cone detection) timestamp so the tracker
# can measure end-to-end latency; the tracker only acts on fresh paths.
std_msgs/Header header            # frame: map frame (e.g. "track")
geometry_msgs/PoseStamped[] poses # path points in the map frame
float64[] target_speeds           # [m/s], same length as poses
float64[] curvatures              # [1/m], signed (left turn positive)
