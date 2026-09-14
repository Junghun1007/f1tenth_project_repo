# auto_control

`auto_control_node` follows the yellow centerline published by `line_detactor`
as `line_detactor/msg/LaneResult`. It writes to the VESC duty, brake-current,
and servo topics used by manual driving. Do not run manual and auto driving together.
The installed runtime is C++/`rclcpp`; Python and NumPy are not on the per-frame
control path. The package's older Python files remain only as implementation
reference and are not installed by the `ament_cmake` build.

## 개발 중인 신호등 정지 기능 (기본 OFF)

`auto_control_test.yaml`의 `auto_control.ros__parameters`에 설정한다.
검출 확신도/FPS YAML은 `traffic_light_test.yaml`로 계속 분리한다.

```yaml
traffic_stop_enabled: false
traffic_state_topic: "/traffic_light/state"
traffic_state_timeout_sec: 0.30
traffic_stop_line_timeout_sec: 0.50
traffic_stop_margin_m: 0.15
traffic_front_axle_to_bumper_m: 0.10
traffic_stop_deceleration_mps2: 0.60
traffic_brake_response_time_sec: 0.15
traffic_brake_amps_per_mps2: 2.0
traffic_brake_max_current_amps: 2.5
traffic_hold_current_amps: 0.7
```

값은 시작 시 읽는다. `traffic_stop_enabled: true`로 재실행하거나 기존 launch에
`traffic_stop_enabled:=true`를 추가하면 켜진다. CLI가 YAML보다 우선하며
`traffic_stop_enabled:=false`로 재실행하면 기존 주행 동작으로 돌아간다.
신호등 검출기 자체를 켜는 `traffic_light_enabled`와 별개다.

- 유효 촬영 시각을 가진 RED 수신 시 정지 요구를 유지한다. UNKNOWN/미검출/신호 타임아웃은
  이 요구를 해제하지 않는다. 최신 GREEN은 요구를 해제하고, 새로운 유효 차선/ERPM과
  기존 enabled/drive 조건을 만족할 때 주행을 재개한다. 최초 RED 전 UNKNOWN은 기존 주행을 허용한다.
- 정지선 거리(`LaneResult.stop_line_distance_m`)는 앞차축 기준 경로 거리다.
  범퍼 오프셋, 여유 거리, 영상 촬영 이후 추정 이동량을 빼서 남은 거리를 계산한다.
  기본 범퍼 오프셋 0.10m는 차량 실측값이 아니라 초기 설정값이므로 실제 치수로 바꾼다.
- 남은 거리 d, 계획 감속도 a, 구동기 응답 여유 t에 대해
  `v_limit = sqrt((a*t)^2 + 2*a*d) - a*t`로 속도를 제한한다.
  코너 속도 제한과 비교해 작은 목표 속도를 사용한다. a를 작게 하면 더 일찍 감속한다.
- 현재 속도가 목표보다 높으면 남은 제동 거리와 `v^2/(2*d)`에 비례한 요구 감속도로
  브레이크 전류를 정한다. 전류는 `traffic_hold_current_amps`부터
  `traffic_brake_max_current_amps` 범위로 제한한다. 속도 오차의 작은 히스테리시스로
  제동/해제 전환을 완화한다. 속도/거리→전류 계수는 차량에서 조정해야 한다.
- 신호등 정지에서는 별도 최소 duty=0 PID로 저속 접근한다. 정지 목표를 지나거나
  정지선 미확보로 목표 속도 0에서 정차하면 GREEN까지 정차 상태를 유지한다.
  정차 목표에서는 양의 duty를 발행하지 않고 제동 전류를 계속 갱신한다.
- 정지선 미검출은 0.50초까지 이동량으로 보정한다. RED인데 정지선 거리가 없거나
  장기간 사라지면 즉시 목표 속도를 0으로 낮춘다. 빨간불 중 목표가 더 먼 정지선으로
  바뀌지 않게 거리는 보수적으로 갱신하므로 노이즈에 의해 일찍 멈출 수 있다.
- RED 중 차선/ERPM이 끊기면 watchdog도 제동을 유지한다(설정 전류 상한).
  `enabled=false`, VESC 단절, steering_only/monitor_only는 기존 출력 제한을 우선한다.
  개발용 기능이며 내 차로 신호등 선택/딜레마존 판단은 없다.

**신호등 정지용 제동은 `electrical_brake_enabled`와 독립적**이다. 일반 코너 감속
브레이크를 꺼도 이 기능을 켜면 신호등 정지에서 전류를 명령한다. VESC bridge의 전류
상한도 함께 적용된다. 계획 감속도와 실제 감속도는 같다고 보장할 수 없고, 전기 제동은
경사면에서 기계식 주차브레이크와 같은 정지 유지를 보장하지 않는다.
이번 변경의 빌드/실차/자동 테스트는 수행하지 않았다.

메시지 형식이 추가되었으므로 검출기와 제어기를 함께 다시 빌드해야 한다:

```bash
colcon build --packages-up-to vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

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

For every valid centerline that reaches final servo/duty calculation, the node publishes
`/auto/detector_input_to_control_decision_ms`. It starts when the detector receives the
BEV image and ends immediately before actuator command publication. Camera-image transport
before detector input, actuator command publication, and VESC UART transmission are excluded;
the detector queue and `LaneResult` transport are included. The line-detector preview uses
these samples for its cumulative `control avg` time and reciprocal FPS.

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
`stop_line_present` is diagnostic and does not trigger automatic stop-line handling.

The control result carries timestamps, lane state/geometry, and the ordered
centerline only. BGR/label/mask payloads are not serialized through the
latency-sensitive control topic. Use `/line_detactor/result_image` for visual
diagnostics; it is generated only while the preview or an image subscriber needs it.

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
Diagnostic messages are serialized only while a subscriber is present. VESC
actuator command publication is unconditional in the corresponding active mode.
