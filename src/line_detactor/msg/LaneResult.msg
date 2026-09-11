# Same timestamp and source frame_id as the input BEV. Pixel mapping is explicit below.
std_msgs/Header header
# Per-frame performance metadata. detector_input_received_stamp is when the
# line detector received the BEV image; detector_result_ready_stamp is taken
# immediately before publishing this result. All durations exclude GUI work.
builtin_interfaces/Time detector_input_received_stamp
builtin_interfaces/Time detector_result_ready_stamp
string engine_precision
uint64 detector_sequence
uint64 detector_queue_nanoseconds
uint64 h2d_preprocess_nanoseconds
uint64 pure_inference_nanoseconds
uint64 label_export_nanoseconds
uint64 backend_postprocess_nanoseconds
uint64 lane_geometry_nanoseconds
uint64 result_message_build_nanoseconds
uint64 detector_total_compute_nanoseconds
uint8 NONE=0
uint8 LEFT_ONLY=1
uint8 RIGHT_ONLY=2
uint8 BOTH=3
# State means retained label presence, not a validated single curve per side.
uint8 state
uint8 BORDER_ONLY=1
uint8 processing_mode
uint32 source_width
uint32 source_height
uint32 padding_left
uint32 padding_right
# Original pixel: u_source = u_result - padding_left; v_source = v_result.
# Negative/out-of-source coordinates are inferred, not newly observed camera pixels.
# Diagnostic segments for border components and added bridges only.
# labels are authoritative for lanes; stop_line_mask independently represents stop lines.
# image composites lanes, stop lines and the yellow centerline. Disconnected in-frame components are preserved.
line_detactor/LaneCurve left
line_detactor/LaneCurve right
# No camera texture/banner. Extended dimensions, black background.
sensor_msgs/Image image
# mono8: 0=background/unknown, 1=left model, 2=right model, 3=left bridge, 4=right bridge.
# Background means no lane label; it does NOT indicate free space.
sensor_msgs/Image labels
# Independent stop-line membership (0/255 mono8), same padded canvas as image/labels.
# Preserves left/right membership at intersections. Padding is always zero.
# Model prediction only; no stop-line interpolation or connected-component filtering.
sensor_msgs/Image stop_line_mask
# True iff stop_line_mask contains >=1 positive pixel; not a vehicle stop command.
bool stop_line_present

# Yellow center path, independent of labels and stop_line_mask.
# Pixels in the padded image, ordered from near vehicle to far along the path.
# valid only means a geometric path exists, NOT a validated driving command.
bool centerline_valid
bool centerline_sample_limit_reached
geometry_msgs/Point32[] centerline_points
uint8 CENTER_SINGLE=1
uint8 CENTER_PAIRED=2
uint8 CENTER_GAP=3
uint8 CENTER_OUTER=4
# Same length as points. Local construction support, not a probability.
# Short graph edges and subsequent smoothing can mix neighboring support.
uint8[] centerline_support
# mono8 0/255, same padded canvas as image; zero when disabled/no path.
sensor_msgs/Image centerline_mask
# Physical extent of the unpadded source used for centerline geometry.
float32 centerline_bev_width_m
float32 centerline_bev_height_m
