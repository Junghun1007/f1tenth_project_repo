# auto_control

`auto_control_node` follows the drive centerline published by `bev_processor`
and writes directly to the same VESC duty, brake-current, and servo topics used
by manual driving. Do not run `manual_drive.launch.py` at the same time.

## Control pipeline

1. Subscribe to the ordered center/left/right `nav_msgs/Path` outputs from
   `bev_processor`. `path_input_mode:=image` retains the previous mono8 mode.
2. Preserve arc-length order through horizontal sharp corners instead of
   reducing the path to one point per image row.
3. Measure left/right boundary clearance independently, correct a biased BEV
   center where both boundaries are valid, and build a vehicle-width-aware
   corridor. No equal boundary-curvature assumption is made.
4. Apply a smooth, corridor-clamped outside-inside-outside offset and publish
   the path actually followed as `/auto/planned_path`.
5. Calculate Stanley cross-track error as the signed normal distance from the
   real front-axle origin and heading from a short local centerline chord.
6. Build a spatial curvature speed limit, run a backward braking pass, and
   reduce available longitudinal deceleration toward the apex using a combined
   lateral/longitudinal acceleration constraint.
7. Convert VESC measured ERPM to vehicle speed and apply PID plus a linear
   duty feed-forward within the configured bounds.
8. When measured speed exceeds target by the configured entry threshold,
   replace duty control with ramp-limited `COMM_SET_CURRENT_BRAKE`. Release at
   the lower exit threshold, send brake current zero for one cycle, and only
   then resume positive duty.

The drive command becomes exactly zero and steering returns to center when the
centerline is missing/short/stale, measured ERPM is stale, the VESC reports a
disconnect, the node is disabled, or the node shuts down. When all inputs are
valid, the configured `0.070` start duty is applied in the same way as manual
driving and subsequent changes are rate limited.

Automatic electrical braking defaults to a conservative `2.5A` maximum. It
enters at `current_speed - target_speed >= 0.10m/s`, remains active down to
`0.03m/s`, and is disabled below `0.20m/s`. Invalid/stale input safety stops
release brake current and send duty zero; they do not command an emergency
brake.

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

Supply independent test YAML files to the two nodes. Explicit launch arguments
after these paths override the selected YAML values for that run:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  bev_params_file:=/path/to/bev_test.yaml \
  auto_control_params_file:=/path/to/auto_control_test.yaml \
  local_path_racing_line_weight:=0.20
```

Start disarmed for actuator/diagnostic checks:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  preview_enabled:=false \
  auto_enabled:=false
```

Tune electrical braking without editing YAML:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  brake_entry_speed_error_mps:=0.12 \
  brake_exit_speed_error_mps:=0.04 \
  brake_maximum_current_amps:=3.0 \
  brake_current_gain_amps_per_mps:=6.0 \
  brake_current_rise_amps_per_sec:=6.0 \
  brake_current_fall_amps_per_sec:=16.0
```

Set `electrical_brake_enabled:=false` to compare against the previous
duty-only speed control.

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

The complete Korean symptom-based tuning guide is installed as
`share/auto_control/AUTO_CONTROL_PARAMETER_TUNING_KO.txt`. Corridor, racing
line, and spatial-speed parameters are covered separately in
`share/auto_control/LOCAL_PATH_PLANNER_PARAMETER_TUNING_KO.txt`.

- `local_path_corner_approach_length_m`: distance used to move from the
  center toward the outside setup position.
- `local_path_corner_setup_transition_length_m`: explicit replacement for
  the approach transition; zero keeps legacy `corner_approach_length_m`.
- `local_path_corner_pre_turn_outside_hold_m`: distance that the outside
  setup position is held before turn-in begins.
- `local_path_post_corner_offset_hold_distance_m`: measured travel distance
  that preserves the outside exit position after the corner leaves the BEV.
- `local_path_same_direction_corner_link_maximum_gap_m`: maximum gap over
  which same-direction corners share one continuous outside setup line.
- `stanley_gain`: larger values correct lateral displacement more strongly.
- `stanley_heading_lookahead_m`: larger values use a farther, smoother heading.
- `path_local_smoothing_window_m`: larger values reject wider centerline
  roughness but can soften a very tight corner.
- `path_outlier_threshold_m`: smaller values reject smaller isolated lateral
  bumps; it does not change a continuous corner.
- `path_geometry_window_m`: larger values make local heading and curvature less
  sensitive to centimetre-scale steps.
- `path_minimum_x_m`, `path_minimum_points`, `path_minimum_span_m`: minimum
  directly measured centerline coverage accepted by the controller. Smaller
  values keep tight, mostly lateral corners valid but reduce path confidence.
- `stanley_corner_heading_threshold_deg`: enables the corner direction guard
  above this absolute path heading.
- `stanley_corner_opposing_correction_ratio`: limits an opposing cross-track
  correction so it cannot reverse the path-heading steering direction.
- `steering_current_weight`: smaller values smooth steering more but add lag.
- `steering_servo_inverted`: reverses only the final servo output while keeping
  positive `/auto/steering_angle_rad` defined as a vehicle-left command.
- `maximum_lateral_acceleration_mps2`: smaller values reach the minimum-speed corner
  limit on gentler curves.
- `curvature_percentile`: smaller values ignore more isolated curvature spikes.
- `speed_pid_kp`, `speed_pid_ki`, `speed_pid_kd`: measured-speed PID gains.
- `brake_entry_speed_error_mps`: overspeed required to enter electrical
  braking; increase it if braking triggers too often.
- `brake_exit_speed_error_mps`: lower hysteresis boundary used to release the
  brake; it must stay below the entry threshold.
- `brake_minimum_current_amps`, `brake_maximum_current_amps`: brake-current
  bounds while braking is requested.
- `brake_current_gain_amps_per_mps`: converts speed overshoot into amperes.
- `brake_current_rise_amps_per_sec`, `brake_current_fall_amps_per_sec`: limit
  brake application and release rates.
- `speed_scale_correction`: calibrates ERPM speed using measured travel distance.

Diagnostic topics are `/auto/current_duty`, `/auto/current_brake_current`,
`/auto/target_speed`, `/auto/current_speed`, `/auto/path_curvature`, and
`/auto/steering_angle_rad`. The ordered inputs are `/camera/path_bev_lane`,
`/camera/path_bev_lane_left`, and `/camera/path_bev_lane_right`; the selected
route is `/auto/planned_path`. Stanley steering diagnostics are available at
`/auto/cross_track_error_m`, `/auto/heading_error_rad`,
`/auto/raw_steering_angle_rad`, and `/auto/current_servo_position`.
