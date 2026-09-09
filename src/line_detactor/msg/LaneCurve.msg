# Points are extended-image PIXELS (x right, y down, z=0), not metres/TF positions.
# Border diagnostics only; does NOT enumerate all retained model pixels/components.
# Points/provenance have identical lengths. Each segment is independent.
# NEVER connect across segment_starts; no single-lane-chain guarantee.
uint8 MODEL=0
uint8 BRIDGE=1
geometry_msgs/Point32[] points
uint8[] provenance
uint32[] segment_starts
# Amount of original skeleton support; does not count interpolated length.
float32 observed_length_px
