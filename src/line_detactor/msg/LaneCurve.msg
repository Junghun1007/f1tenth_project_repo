# Points are extended-image PIXELS (x right, y down, z=0), not metres/TF positions.
# Ordered from near to far. Empty means absent. Arrays always have identical lengths.
uint8 MODEL=0
uint8 BRIDGE=1
geometry_msgs/Point32[] points
uint8[] provenance
# Amount of original skeleton support; does not count interpolated length.
float32 observed_length_px
