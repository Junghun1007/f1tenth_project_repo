# auto_control

## Front-camera lane detector

The front-lane detectors use the rectified **front-camera image directly**. They
does not create a bird's-eye-view image and does not use stereo depth.

The node detects low-saturation bright lane markings in a configurable lower
image ROI, follows each marking with independent sliding windows, fits a
quadratic image-space curve, and applies short temporal confirmation/hold to
avoid background-induced jumps.  If only one marking is visible, it retains
that marking and estimates the centerline from the last reliable lane-width
profile rather than forcing a second detection.

Input and outputs:

```text
/camera/image_rect       sensor_msgs/Image (NV12 from camera_driver)
/front_lane/mask         sensor_msgs/Image (mono8 candidate mask)
/front_lane/overlay      sensor_msgs/Image (bgr8; detected lanes are blue)
/front_lane/model        std_msgs/Float32MultiArray
```

`/front_lane/model.data` is:

```text
[confidence, lateral_error_normalized, lookahead_offset_normalized,
 curvature_px_inverse, left_detected, right_detected]
```

The lookahead offset and curvature are intentionally published for a later
controller.  This first node never publishes a VESC or steering command.

Run the camera and current rotated-window detector together:

```bash
ros2 launch auto_control front_lane.launch.py
```

The current launch opens one OpenCV result window and caps it at 30 FPS. Its
resizable dimensions are configured with `preview_window_width_px` and
`preview_window_height_px`. Set `preview_enabled: false` when operating
headlessly, then inspect `/front_lane/overlay` with `rqt_image_view` if needed.

## Rotated-window detector

`front_lane_rotated_detector` starts from the detected seed closest to the
vehicle and follows each boundary only toward the far field. Every search
window is aligned with the locally measured lane tangent. Its initial width
and height, minimum size, shrink ratio and step ratio are configurable in
`config/front_lane_rotated_detector.yaml`.

The rotated detector opens only the shared result preview. Detected boundaries,
the lane centreline and the true rotated search polygons are drawn together in
that window.
