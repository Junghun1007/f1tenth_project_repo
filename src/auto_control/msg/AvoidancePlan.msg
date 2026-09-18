# Metric path in front_axle_bev (+X forward, +Y left), NOT padded pixels.
# header is the source lane capture, depth_stamp the matched depth capture.
std_msgs/Header header
builtin_interfaces/Time depth_stamp
bool control_ready
bool valid
string status
float64 speed_limit_mps
float64 max_curvature_per_m
geometry_msgs/Point32[] points
