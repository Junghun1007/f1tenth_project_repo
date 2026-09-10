# Same timestamp and source frame_id as the input BEV. Pixel mapping is explicit below.
std_msgs/Header header
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
# image composites both. Disconnected in-frame components are preserved.
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
