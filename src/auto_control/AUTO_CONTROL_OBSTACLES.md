# 기존 auto_control 통합 실행에서 장애물 매핑 확인

기존 `vehicle_bringup/auto_drive.launch.py`에 장애물 검출을 연결했다.
**차선 검출 + 신호등 검출 + 장애물 BEV 매핑**을 함께 실행하며,
`manual_test:=true`로 자동 제어 출력을 끄고 조이스틱으로 수동주행할 수 있다.
BEV는 기존 **120cm × 전방 300cm**, 딥러닝 차선 모델 입력은 **120×300px**를 유지한다.

## 이번 수동 실험 실행

차량 PC의 저장소 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=true \
  obstacle_params_file:="$(pwd)/src/auto_control/config/obstacles.yaml"
```

평지에서 정지한 상태로 시작 측정과 모델 준비를 끝낸 후 수동주행한다.
이 실행은 수동조작·VESC·운동 모니터도 함께 켠다. 기존 수동주행이 이미 실행 중이면
`manual_enabled:=false`를 추가한다. 운동 모니터도 실행 중이면 `dynamics_enabled:=false`를 추가한다.
수동 차량 namespace 기본값은 `autopilot03`이며 `vehicle_namespace:=...`로 변경할 수 있다.
기존 카메라/auto_drive 실행은 종료하고 이 명령 하나만 사용한다.

`manual_test:=true`는 기존 `auto_control_node`를 `monitor_only`로 유지한다. 차선 추종 계산과
진단은 계속하고 조향·구동·제동 명령을 발행하지 않으므로 수동조작과 충돌하지 않는다.
외부 auto_control YAML이 drive여도 수동 실험 옵션이 우선한다. 신호등 정지 제어는 OFF이고,
성능 측정용 자동 종료/GUI 비활성화도 해제한다. `auto_enabled=false`만 사용하는 방법은
정지 명령이 수동조작에 섞일 수 있어 이 실험에서는 monitor_only를 사용한다.

## 구성과 표시

- 기존 `camera_driver`가 한 OAK에서 RGB와 스테레오 Depth를 함께 받는다.
- `bev_processor`는 기존 영상 변환과 시작 측정/기하 공유를 담당한다.
- **`auto_control::ObstacleDetectorNode` (`/auto_obstacles`)**가 별도 스레드에서
  ROI 깊이 → 바닥 제거 → 각도별 거리 → 군집 검출을 수행한다. 제어 루프와는 별도로 동작한다.
- 기존 `line_detactor` 딥러닝 모델과 `traffic_light_detector`를 사용한다. 신호등은
  BEV 변환 전 RGB를 공유하고, 장애물은 같은 프로세스의 Depth를 공유한다.
- 기존 차선 결과 프리뷰에 군집 외곽선·대표점·거리, 하단에 Depth FPS·시각 차이와
  `Traffic: RED / GREEN / UNKNOWN`이 표시된다.
- 장애물 위치 토픽은 `/auto/obstacles` (PoseArray)다. XY는 앞차축 기준 관측 표면의 평균,
  Z=0, 방향은 추정하지 않는다. 화면 합성에는 군집 표면점도 내부적으로 전달한다.
- 장애물 외곽선은 관측 표면의 볼록 외곽선이고 물체 전체 영역을 복원한 것은 아니다.
  색상/#번호는 프레임 안에서의 구분이며 추적 ID가 아니다.

기본은 장애물 표시만 수행한다. **`avoidance_control_enabled:=true`를 명시하면 검증된
회피 경로를 실제 조향·속도 제어에 사용하고, 계획이 없으면 제동한다.**
`manual_test:=true`는 이 옵션을 켜도 monitor_only를 유지하므로 액추에이터 출력은 없다.
독립 `roi_lidar` 패키지는 유지하며 이 통합 실행에서는 시작하지 않는다.

## YAML 조정

`src/auto_control/config/obstacles.yaml`을 직접 수정하고 launch를 재시작한다.
위 명령처럼 소스 YAML 경로를 넘기면 YAML 변경 후 재빌드가 필요 없다.
네 섹션은 모두 `obstacles.*` 옵션만 허용한다.

| YAML 섹션 | 담당 |
|---|---|
| `camera_driver` | Depth 해상도/FPS/IR 강도·신뢰도·LR/subpixel |
| `bev_processor` | `obstacles.share_reference`: 시작 기하 공유 |
| `auto_obstacles` | ROI, 바닥 제거, 스캔·군집 및 동기화 |
| `line_detactor` | 장애물 표시·촬영 시각 차이·만료 |

실행 시 `obstacles_enabled`가 카메라 Depth·기하 공유·검출 노드·화면 합성을 한 번에 켜고 끈다.
기본 true이며 부하 비교는 `obstacles_enabled:=false`로 한다.
차선·신호등·제어의 기존 파일은 각각 `line_detactor_params_file`, `traffic_light_params_file`,
`auto_control_params_file`로 지정한다. 신호등은 기존 `traffic_light_enabled` 옵션을 유지한다.

작은 장애물은 `obstacles.points.pixel_stride: 1`로 표본을 늘리고,
`obstacles.cluster.min_bins: 1`로 작은 묶음을 허용할 수 있다. 연산량/노이즈가 증가할 수 있다.
바닥 제거 두께는 `min(max_distance_m, distance_m + distance_per_meter × 수평거리)`다.
기본 1m에서 6cm, 3m에서 8cm 이하를 제외하고 군집 방향에는 10cm 이상의 높이 근거를 요구한다.
낮은 장애물도 제외될 수 있다. `obstacles.avoidance.enabled` 이외의 장애물 옵션은
시작 시 적용하며 YAML 저장 후 재실행한다.

## 시간·성능·검증 범위

시작할 때 한 번 측정한 평면과 RGB에 실제 사용한 고정 줌/자세 보정, 프레임 내부·외부
보정을 사용해 동일 BEV 좌표로 변환한다. Depth와 RGB 보정의 촬영 시각 차이는 기본 40ms,
차선 화면과 장애물은 60ms 이내로 제한한다. 촬영 후 250ms가 지난 군집은 GUI에서 지운다.
`depth WAIT / stale or unsynced`는 신선한 대응 결과를 기다리는 상태다.
차량 이동/Depth 시각으로 자세를 별도 보간하지 않으므로 주행 중 정합은 실측이 필요하다.

카메라/Depth 큐는 최신 한 장을 유지하며 전체 포인트 클라우드의 ROS 직렬화를 하지 않는다.
차선/신호등은 GPU, 군집은 CPU에서 동작하나 USB·메모리·GPU 자원 경쟁은 남는다.
Depth 기본 요청은 400p/60FPS다. 실제 속도는 화면과 `OBSTACLES` 로그에서 확인한다.
코드 수정에서는 요청대로 빌드·테스트·실차 실행을 하지 않았다.

## 실제 회피 제어 연결 (실험 기능, 기본 OFF)

`avoidance_test_enabled`는 화면용이고 **`avoidance_control_enabled`가 실제 제어 적용 스위치**다.
새 `auto_control/msg/AvoidancePlan` 메시지를 추가했으므로 기존 설치에서 재빌드가 필요하다.

```bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 기존 실행을 종료한 뒤 사용. 아래 명령은 실제 조향/구동/제동을 활성화한다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=false \
  avoidance_control_enabled:=true \
  auto_control_mode:=drive \
  auto_enabled:=true \
  obstacle_params_file:="$(pwd)/obstacles_test.yaml" \
  auto_control_params_file:="$(pwd)/auto_control_test.yaml" \
  bev_params_file:="$(pwd)/bev_config_test.yaml" \
  line_detactor_params_file:="$(pwd)/line_detactor_test.yaml" \
  traffic_light_params_file:="$(pwd)/traffic_light_test.yaml" \
  input_mode:=slcan slcan_channel:=/dev/ttyACM0 slcan_bitrate:=500000 \
  can_controller_id:=112 preview_enabled:=true preview_result_only_enabled:=true
```

- 실제 적용 OFF: 재실행할 때 `avoidance_control_enabled:=false`. 기존 차선 추종으로 돌아간다.
- 실제 적용 알고리즘을 구동 없이 확인: `manual_test:=true avoidance_control_enabled:=true`.
  VESC/조이스틱/운동 모니터도 필요 없으면 `manual_enabled:=false dynamics_enabled:=false` 추가.
- 기존 화면 전용: `manual_test:=true avoidance_control_enabled:=false avoidance_test_enabled:=true`.
- 적용 ON에서는 계획 계산도 자동으로 ON. `avoidance_test_enabled:=false`로 실제 적용을 해제할 수 없다.
- 실행 중 계획 OFF는 아래와 같이 가능하지만 **적용 ON 상태에서는 정지 요청**이다.
  중앙 경로로 몰래 복귀하지 않는다. 실제 적용 여부는 시작 시 고정(read-only)이다.

```bash
ros2 param set /auto_obstacles obstacles.avoidance.enabled false
ros2 param set /auto_obstacles obstacles.avoidance.enabled true
ros2 topic echo /auto/avoidance_plan
ros2 topic echo /auto/avoidance_preview/status
```

### 제어 흐름과 중단 조건

`auto_obstacles`가 미터 좌표 경로와 차선/Depth 촬영 시각을 발행하고, `auto_control`은
프레임·시간·점 순서·간격·시작 위치·곡률을 다시 검사한 뒤 기존 Stanley 조향 계산에 넣는다.
기존 차선 경로와 신호등 정지 입력은 계속 수신하지만, 회피 적용 중 조향 경로를 차선 중앙으로
대체하지 않는다. 차선 자체가 끊겨도 정지하며 기존 경로 유지 기능은 적용 모드에서 사용하지 않는다.

회피 경로가 없거나 WAIT/BLOCKED/OFF/잘못된 값/만료이면 구동 0과 제동 전류를 요청한다.
ROS 시각뿐 아니라 수신 후 steady-clock 경과도 확인하며, planner 노드가 죽거나 메시지가
멈춰도 기존 watchdog이 제동한다. 회복에는 새 Depth를 포함한 유효 계획 3회가 연속 필요하다.
일반 주행의 `electrical_brake_enabled=false` 설정으로 회피 정지 제동이 꺼지지 않는다.
신호등 정지가 활성화되어 있으면 속도는 두 제한 중 작은 값을 따르고, 입력 실패 시 제동은
신호등/회피 제동 중 큰 값을 사용한다. 신호등 검출 ON과 신호등 정지 제어 ON은 별도 설정이다.

최대 조향각/축간거리로 가능한 곡률을 제한한다. 기본 목표 속도 상한은 **0.4m/s**이며,
기존 test YAML의 minimum_speed=0.8 같은 값이 이 상한을 올리지 않는다. 회피 주행에는
최소 duty/출발 부스트를 적용하지 않는 signed PID를 사용한다. 목표 상한보다 0.15m/s 이상
빠르면 제동 요청으로 전환한다. 이는 실측 속도를 즉시 0.4로 만드는 보장이 아니다.

### 13×13cm 장애물과 파라미터

기존 `obstacles_test.yaml`의 `auto_obstacles.ros__parameters`에 필요하면 추가한다.
생략해도 코드 기본값은 0.13이다.

```yaml
    obstacles.avoidance.obstacle_size_m: 0.13
```

Depth는 물체 중심이 아니라 앞 표면을 보므로, 관측 표면 뒤로 13cm를 확보하고 최소 가로 폭도
13cm로 잡는다. 부분 관측의 횡방향 위치 오차 여유는 `unknown_extent_m`와 반폭 6.5cm 중
큰 값을 쓴다. 여기에 실제 차량 반폭·반길이·안전 여유를 적용한다. 붉은 영역은 단순히
13cm 정사각형이 아니라 차량 중심이 피해야 하는 확장 영역이다.

`auto_control_test.yaml`의 `auto_control.ros__parameters`에 추가 가능한 제어 설정:

```yaml
    avoidance_control_enabled: false  # launch의 명시값이 우선
    avoidance_max_age_sec: 0.20
    avoidance_speed_cap_mps: 0.4
    avoidance_brake_current_amps: 2.5
    avoidance_wheelbase_m: 0.33
```

차량 축간거리/차체 크기/제동 전류는 실차에 맞춰야 한다. 제어 옵션은 모두 시작 시 고정이다.
기존 외부 YAML에 이 항목들이 없어도 기본값으로 동작한다. 실제 적용 시 planner의 속도·곡률·
유효 시간 상한은 제어기 설정과 비교해 더 작은 값으로 맞춘다.

`avoidance_speed_cap_mps`의 허용 범위는 0 초과 ~ 1.0m/s 이하이다. 1.0m/s까지
목표 속도를 허용하려면 제어 YAML의 이 값과 장애물 YAML의
`obstacles.avoidance.max_speed_mps`를 모두 1.0으로 설정하고 일반 속도 상한도 확인한다.
곡률/신호등 제한은 계속 적용되므로 항상 1.0m/s로 주행한다는 뜻은 아니다.
허용 범위 확장은 실차에서 해당 속도의 회피·제동 성능을 검증했다는 뜻이 아니다.

| `obstacles.avoidance.*` | 기본값 | 의미 |
|---|---:|---|
| `enabled` | false | 계산 ON/OFF. 실시간 변경 가능, 적용 중 OFF는 정지 |
| `max_fps` | 10.0 | 계산 주기 상한, 허용 1..30. Depth FPS와 별개 |
| `max_age_sec` / `max_sync_sec` | 0.25 / 0.06 | 입력 만료 / RGB-Depth 시각 차이. 적용 시 만료는 최대 0.20 |
| `vehicle_half_width_m` / `vehicle_half_length_m` | 0.15 / 0.25 | 앞차축 기준 차체 좌우/전후 최대 거리 |
| `obstacle_size_m` | 0.13 | 관측 물체의 최소 가로 폭과 표면 뒤 깊이 |
| `safety_margin_m` / `unknown_extent_m` | 0.04 / 0.04 | 차량 여유 / 물체 관측 오차 여유 |
| `sample_step_m` | 0.025 | 경로 샘플 간격 |
| `max_offset_m` / `offset_step_m` | 0.40 / 0.05 | 좌우 이동 최대값 / 후보 간격 |
| `transition_m` | 0.70 | 최초 회피 최소 접근 길이, 공간이 있으면 완만하게 사용 |
| `max_curvature_per_m` | 2.0 | 최대 곡률. 적용 시 차량 조향 한계로 추가 제한 |
| `max_speed_mps` | 0.5 | 계획 속도. 적용 시 기본 제어 상한 0.4로 추가 제한 |
| `lateral_acceleration_mps2` | 0.4 | 곡률에 따른 속도 제한 |
| `deceleration_mps2` | 0.5 | 화면 전용 모드의 접근 권고 속도 계산 |
| `clear_confirm_sec` | 1.0 | 중앙 경로 복귀 전 연속 관측 시간 |

### 화면과 현재 제한

노랑=기존 중앙 경로, 청록=선택 경로, 흐린색=후보, 붉은 영역=차량 크기를 반영한 장애물 영역.
`APPLY`/`CONTROL PATH REQUESTED`는 제어 적용 설정이며, `manual_test`/`auto_enabled` 등
최종 구동 조건까지 충족해야 실제로 움직인다. `TEST`는 화면 전용이다.

적용 모드 경로는 차량 현재 위치/방향에서 출발하도록 보정하고, 남은 전방 시야 안에서
복귀 공간이 부족하면 옆으로 피한 상태를 유지하는 후보도 만든다. 모든 후보에 대해 관측
양쪽 차선, 다른 군집, 회전 방향을 반영한 차체 캡슐, 곡률을 검사한다. 좁은 차선에서는
13cm 장애물이라도 회피 공간이 없을 수 있으며 이때 정지가 정상이다.

회피 대상으로 보던 물체가 시야에서 사라지면 **통과했다고 간주하지 않고 BLOCKED**로 둔다.
후방 센서/차량 이동 보정에 의한 통과 검증이 없어, 통과 끝에서 정지할 수도 있다.
차량 정지와 주변을 확인한 뒤 planner를 OFF→ON하면 이 보류 상태를 초기화할 수 있다.
물체가 다시 관측되면 정상 검증을 거쳐 회복한다. 이 제한을 없애려면 별도 추적/이동 보정이 필요하다.

계획은 최대 200ms 안의 입력만 제어에 사용한다. 이동 오차를 줄이기 위해 저속 상한과
전방 이동/곡률 여유를 추가하지만, odometry로 과거 경로를 현재 차량 위치에 변환하는 구현은
없다. 화면은 계획 주기에 맞춰 직전 경로를 잠깐 재사용하나 제어 유효 시간 검사는 별도로 한다.
바닥 잔여점·미검출·보정 오차와 실제 제동 성능은 코드만으로 확인할 수 없다.
이번 변경은 요청대로 빌드·테스트·실차 실행 없이 작성했으며 실제 회피 성공은 검증하지 않았다.
