# auto_control

`auto_control_node` follows the yellow centerline published by `line_detactor`
as `line_detactor/msg/LaneResult`. It writes to the VESC duty, brake-current,
and servo topics used by manual driving. Do not run manual and auto driving together.

## Control pipeline

Control is triggered by each received `LaneResult`: validate the path, calculate
Stanley/PID and publish duty/brake/servo commands directly in that callback.
The controller does not wait for an 80Hz timer or repeatedly update PID on an old image.
`control_rate_hz` is the stop-watchdog rate and reference rate for the configured
steering filter weight. PID, brake ramps and steering/duty rate limits use elapsed
time between results, bounded by the input freshness limits. Empty or invalid results
immediately send stop commands. With no new results, the watchdog still stops on
stale path/ERPM, disconnect or disable. Enabling or recovering telemetry alone does
not initiate motion; a new valid lane result must arrive.

The ROS command is published immediately after calculation. VESC UART transmission
continues through the existing bridge worker; this is not a hard real-time guarantee.

1. Subscribe to `/line_detactor/result`; validate the source timestamp, frame,
   centerline validity, sample limit, point/support arrays, and metric scale.
2. Remove `padding_left` from result pixels and convert pixel centers to metres:
   `X = x_max - (v + 0.5) * mpp`, `Y = y_max - (u - padding_left + 0.5) * mpp`.
3. Preserve near-to-far point order, including sideways corners. Keep the first
   contiguous forward-X ROI segment; stop at an excessive adjacent-point gap.
   `path_minimum_span_m` measures distance along the path, not its X extent.
   Spatial smoothing and outer-boundary weighting happen in `line_detactor`.
4. Calculate Stanley cross-track error from the front-axle origin and heading
   at the closest path projection plus `stanley_heading_lookahead_m` along the path.
5. Estimate curvature using three points spaced along the path and take the
   configured percentile in the forward-X lookahead region (available path if empty).
6. Apply curvature-based target speed, ERPM feedback, PID and duty limits from YAML.
   Optional electrical braking replaces positive duty during overspeed.

Missing, invalid, short, duplicate, out-of-order or stale centerlines invalidate
control. Stale ERPM, VESC disconnect, disable and shutdown also send duty zero
and centered steering. These stops release electrical brake current. Existing
freshness thresholds are retained; camera capture age includes ML inference time.
`stop_line_mask` is diagnostic and does not trigger automatic stop-line handling.

Default BEV is 120x300, X=0..3m and Y=-0.6..0.6m. The integrated launch checks
that output size matches the static model and derives producer/consumer geometry
from the BEV YAML. Direct node launches must provide the same geometry and frame.

Vehicle conversion defaults: 109.5mm tires, two motor pole pairs, 13/54 motor
gearing, and 13/37 differential gearing. Speed, duty and brake defaults are listed
in `config/auto_control.yaml`; electrical braking defaults to disabled.

## Launch

The default autonomous launch shows only the measured left/right lanes and
yellow centerline, without the original camera image:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py
```

Existing BEV/controller/camera YAML arguments and CAN arguments are supported.
Add `line_detactor_params_file:=/absolute/path/line_detactor_test.yaml` to tune
65cm lane width, yellow-line smoothing and outer-boundary weighting. See
[the full launch command](../vehicle_bringup/README.md#autonomous-driving).

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

Measure lateral and heading errors while pushing the vehicle by hand without
publishing duty, brake-current or servo actuator commands:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  auto_control_mode:=monitor_only
```

Use `auto_control_mode:=steering_only` to publish automatic servo commands while
leaving duty and brake-current topics entirely to a manual controller. Passive
modes publish zero motor-command diagnostics but do not publish suppressed VESC
actuator topics. `monitor_only` also computes path diagnostics without requiring
VESC connection or fresh ERPM.

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

Set `electrical_brake_enabled:=true` to enable the configured overspeed braking.

Enable or stop an already running node:

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: true"
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: false"
```

The ML preview does not provide the old BEV Space-key disable shortcut.
Use `/auto/enabled` to explicitly disable control.

The default launch is armed and starts when a valid centerline, VESC connection,
and fresh ERPM have all arrived. Lift the wheels for the first test and keep a
hardware power cutoff reachable.

## Main tuning parameters

The complete Korean symptom-based tuning guide is installed as
`share/auto_control/AUTO_CONTROL_PARAMETER_TUNING_KO.txt`.

- `stanley_gain`: larger values correct lateral displacement more strongly.
- `stanley_heading_lookahead_m`: distance along the path from the closest
  front-axle projection; larger values use a farther heading.
- `path_maximum_gap_m`: stop retaining points at a larger adjacent-point gap.
- `centerline_smoothing_*` in the **line_detactor YAML**: spatial smoothing.
  Legacy `path_local_smoothing_window_m` and `path_outlier_threshold_m` are unused.
- `path_geometry_window_m`: larger values make local heading and curvature less
  sensitive to centimetre-scale steps.
- `path_minimum_x_m`, `path_minimum_points`, `path_minimum_span_m`: minimum
  ML centerline coverage accepted by the controller. Smaller
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
`/auto/steering_angle_rad`. Stanley steering diagnostics are available at
`/auto/cross_track_error_m`, `/auto/heading_error_rad`,
`/auto/raw_steering_angle_rad`, and `/auto/current_servo_position`.
