std_msgs/Header header

# Cropped, tightly packed NV12 source image. The crop keeps the bottom part of
# the full rectified camera frame and is taken before stabilization/BEV warp.
uint32 source_width
uint32 source_height
uint32 source_crop_top
uint32 cropped_width
uint32 cropped_height
uint32 step
uint8[] nv12

# Projective transform from full rectified source pixels to the virtual
# stabilized, fixed-zoom camera image assumed by the fixed BEV LUT.
float64[9] source_to_stabilized_homography
bool dynamic_correction_applied
