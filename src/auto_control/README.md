# auto_control

`auto_control_node` follows the yellow centerline published by `line_detactor`
as `line_detactor/msg/LaneResult`. It writes to the VESC duty, brake-current,
and servo topics used by manual driving. Do not run manual and auto driving together.
The installed runtime is C++/`rclcpp`; Python and NumPy are not on the per-frame
control path. The package's older Python files remain only as implementation
reference and are not installed by the `ament_cmake` build.

## 속도 계획과 PID, 개발 중인 신호등 정지

기본값은 **통합 속도 PID ON, 코너 곡률 감속 OFF, 신호등 정지 OFF**다.
`auto_control_test.yaml`의 `auto_control.ros__parameters`에서 조절하고 재실행한다.
검출 확신도/FPS는 별도의 `traffic_light_test.yaml` 설정이다.

```yaml
longitudinal_pid_enabled: true
curvature_speed_control_enabled: false
longitudinal_pid_kp: 1.0
longitudinal_pid_ki: 0.5
longitudinal_pid_kd: 0.0
longitudinal_pid_integral_limit: 2.0
traffic_stop_enabled: false
traffic_state_topic: "/traffic_light/state"
traffic_state_timeout_sec: 0.30
traffic_stop_line_timeout_sec: 0.50
traffic_stop_margin_m: 0.15
traffic_front_axle_to_bumper_m: 0.10
traffic_stop_deceleration_mps2: 0.60
traffic_brake_response_time_sec: 0.15
traffic_brake_max_current_amps: 2.5
traffic_hold_current_amps: 0.7
traffic_stop_line_match_tolerance_m: 0.30
traffic_stop_line_current_weight: 0.35
traffic_stop_line_confirm_frames: 3
traffic_stop_position_tolerance_m: 0.03
traffic_reapproach_speed_mps: 0.20
```

### 목표 속도 곡선 → 실제 속도 PID → duty/브레이크

코너 감속 OFF에서는 기본 목표가 `maximum_speed_mps`다. Stanley 조향은 그대로 사용한다.
신호등 정지를 켜고 RED가 확인되면, 남은 거리에 따른 속도 곡선 `v(d)`에서 매 차선
결과마다 목표를 다시 계산한다. 하나의 고정 속도나 미리 저장한 배열이 아니라,
정지선에 접근할수록 최종 0까지 낮아지는 연속 곡선을 온라인으로 샘플링한다.

앞차축 기준 정지선 거리에서 범퍼 길이, 여유 거리, 촬영 이후 이동량을 뺀 거리를 d라 할 때
`v(d) = min(주행속도, sqrt((a*t)^2 + 2*a*max(0,d-허용오차)) - a*t)`다.
a는 `traffic_stop_deceleration_mps2`, t는 구동기 응답 여유다. a가 작을수록 일찍 감속한다.
`traffic_stop_position_tolerance_m` 이내에서는 목표를 0으로 한다.
기본 범퍼 길이 0.10m는 실측값이 아니므로 차량 치수로 수정한다.

거리 계산부는 목표 속도만 결정한다. **거리로 제동 전류를 직접 산출하지 않는다.**
통합 PID는 `목표 속도 - 실제 속도`로 -1..1 출력을 계산한다. 양수는 최대 duty의 비율,
음수는 최대 브레이크 전류의 비율로 변환한다. 접근 중 제동 전류 하한은 0이며,
정차 목표 0이고 속도가 0.05m/s 미만일 때만 유지 전류 하한을 적용한다.
duty는 기존 상승/하강 속도 제한을 따르며 최소 duty 강제 적용은 없다.
적분은 출력 포화 시 추가 누적을 막고, 목표 0 진입/복귀와 RED/GREEN 전환에서 초기화한다.
기본 D=0이므로 초기 동작은 PI이고, D를 쓰려면 `longitudinal_pid_kd`로 설정한다.

새 PID 이득은 정규화된 출력용으로, 기존 duty 단위 `speed_pid_kp/ki/kd`와 다르다.
`longitudinal_pid_enabled: false`일 때 일반 주행은 기존 제어로 돌아가지만
신호등 정지를 켜면 RED 감속에는 통합 PID를 사용한다.
`traffic_brake_amps_per_mps2`는 구버전 호환을 위해 읽기만 하며 새 제어에 쓰지 않는다.
일반 주행 PID의 전기 제동은 `electrical_brake_enabled`를 따르고, **RED 정지 제동은
이 옵션과 독립적으로 작동**한다. 양쪽 모두 VESC bridge의 전류 상한이 적용된다.

### 임시 정지와 목표 도달 정차

RED는 정지 요구를 유지하며 UNKNOWN/신호 미검출로 해제되지 않는다. 유효한 최신 GREEN이
해제하고 기존 enabled/drive/차선/ERPM 조건이 충족되면 주행한다. 최초 RED 이전 UNKNOWN은
기존 주행을 허용한다. 정지선 거리 미검출은 `traffic_stop_line_timeout_sec`까지만
이동량으로 보정하고, 그 이후에는 **임시 정지**한다. 차선/속도 입력 손실도 임시 정지다.
정지선이 정상적으로 재확인되고 입력이 복구되면 남은 거리를 따라 저속 재접근한다.
임시 정지를 최종 도착으로 고정하지 않는다. 복구 접근 속도는 GREEN까지
`traffic_reapproach_speed_mps`로 제한한다.

거리 관측은 이동량 보정값과 일치 범위 안에 있는 연속 관측으로 확인한다.
허용 범위 안에서는 증가/감소 양쪽으로 보정하므로 짧게 측정된 거리도 복구할 수 있다.
허용 오차를 넘는 갑작스러운 변화는 두 방향 모두 거부한다. 이는 거리 연속성에 의한
대상 일치 추정이며 정지선의 의미적 ID 추적은 아니다. 큰 추정 오차나 다른 정지선은
자동으로 수용하지 않아 임시 정지가 계속될 수 있다. 정지선 확인 3회는 신호등 색상
연속 확인 설정과 별개이며 한 번의 정지선 오검출을 바로 목표로 고정하지 않게 한다.

최근 확인된 정지선 위치가 허용오차 이내이고, 보정한 위치도 해당 범위이며,
실제 속도가 0.05m/s 미만일 때에만 최종 `position_hold`로 전환한다.
그 상태는 GREEN까지 유지한다. 정지 위치를 지난 경우 뒤로 복귀하지 않는다.
상태 로그 `phase`는 approach/slow_reapproach/position_braking/position_hold/
temporary_stop_line_unavailable/temporary_stop_control_input을 구분한다.
`line_rejected`는 거리 불일치로 거부한 누적 관측 수다.

임시 정지에서 차선/ERPM이 끊기면 watchdog은 설정 전류 상한으로 제동한다.
`enabled=false`, VESC 단절, steering_only/monitor_only는 기존 출력 제한을 우선한다.
실제 제동거리와 새 PID 이득은 실차 검증 전이며, 전기 제동은 기계식 주차브레이크와 같은
경사면 정지 유지를 보장하지 않는다. 이번 변경의 빌드/테스트는 수행하지 않았다.

실행 인자 `traffic_stop_enabled:=true`, `curvature_speed_control_enabled:=false`,
`longitudinal_pid_enabled:=true`로도 설정할 수 있으며 CLI가 YAML보다 우선한다.

```bash
colcon build --packages-select auto_control vehicle_bringup \
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
