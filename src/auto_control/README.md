# auto_control

`auto_control_node` follows the drive centerline published by `bev_processor`
and writes directly to the same VESC duty and servo topics used by manual
driving. Do not run `manual_drive.launch.py` at the same time.

## Control pipeline

1. Subscribe to `/camera/image_bev_lane` (`mono8`).
2. Convert each occupied row to vehicle coordinates (`+X` forward, `+Y` left).
3. Fit a second-order `y(x)` path. This suppresses short centerline wiggles
   before either controller sees the path.
4. Calculate steering with Stanley cross-track and heading errors.
5. Calculate representative forward curvature over `X=0.5..1.6m`.
6. Convert curvature to a `0.8..1.2m/s` target using the configured maximum
   lateral acceleration.
7. Convert VESC measured ERPM to vehicle speed and apply PID plus a linear
   duty feed-forward, bounded to `0.05..0.06`.

The drive command becomes exactly zero and steering returns to center when the
centerline is missing/short/stale, measured ERPM is stale, the VESC reports a
disconnect, the node is disabled, or the node shuts down. When all inputs are
valid, the configured `0.05` start duty is applied in the same way as manual
driving and subsequent changes are rate limited.

Vehicle conversion defaults match `camera_driver`: 109.5mm tire diameter,
two motor pole pairs, 13/54 motor gearing, and 13/37 differential gearing
(approximately 11.82:1 total).

## Launch

The default autonomous launch shows only the measured left/right lanes and
yellow centerline, without the original camera image:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py
```

Run without any GUI preview:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py preview_enabled:=false
```

Start disarmed for actuator/diagnostic checks:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  preview_enabled:=false \
  auto_enabled:=false
```

Enable or stop an already running node:

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: true"
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: false"
```

With the BEV preview focused, pressing `Space` publishes the same disable
command and immediately sends duty zero. It stays disabled until `true` is
published explicitly.

The default launch is armed and starts when a valid centerline, VESC connection,
and fresh ERPM have all arrived. Lift the wheels for the first test and keep a
hardware power cutoff reachable.

## Main tuning parameters

- `stanley_gain`: larger values correct lateral displacement more strongly.
- `stanley_heading_lookahead_m`: larger values use a farther, smoother heading.
- `stanley_corner_heading_threshold_deg`: enables the corner direction guard
  above this absolute path heading.
- `stanley_corner_opposing_correction_ratio`: limits an opposing cross-track
  correction so it cannot reverse the path-heading steering direction.
- `steering_current_weight`: smaller values smooth steering more but add lag.
- `maximum_lateral_acceleration_mps2`: smaller values reach the 0.8m/s corner
  limit on gentler curves.
- `curvature_percentile`: smaller values ignore more isolated curvature spikes.
- `speed_pid_kp`, `speed_pid_ki`, `speed_pid_kd`: measured-speed PID gains.
- `speed_scale_correction`: calibrates ERPM speed using measured travel distance.

Diagnostic topics are `/auto/current_duty`, `/auto/target_speed`,
`/auto/current_speed`, `/auto/path_curvature`, and
`/auto/steering_angle_rad`.
