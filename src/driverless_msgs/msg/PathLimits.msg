# Centerline and track limits produced by urinay.
# path is the centerline; tracklimits_left/right are the boundary points
# corresponding 1:1 to the centerline points.
std_msgs/Header header
bool new_path
bool new_close_midpoints
geometry_msgs/Point[] path
geometry_msgs/Point[] tracklimits_left
geometry_msgs/Point[] tracklimits_right
