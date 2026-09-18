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

장애물로 실제 제어 경로를 바꾸거나 자동 제동하지 않는다. 회피 경로는 아래 테스트 옵션으로
화면에만 표시한다. `manual_test:=false`인 기존 자동주행 모드도 현재는 장애물을 표시만 하므로
장애물 회피 주행이 구현된 것으로 해석하면 안 된다.
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

## 회피 경로 테스트 (기본 OFF, 실제 제어 미연결)

기존 실행 명령에 `avoidance_test_enabled:=true`를 추가한다. 차선/신호등/BEV/제어의
외부 test YAML 인수는 그대로 유지한다. 기존 `obstacles_test.yaml`에 새 항목이 없어도
코드 기본값으로 실행된다. 튜닝하려면 `config/obstacles.yaml`의 `auto_obstacles` 아래
`obstacles.avoidance.*` 항목을 복사한다. 기존 파일 전체를 덮어쓸 필요는 없다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=true \
  avoidance_test_enabled:=true \
  obstacle_params_file:="$(pwd)/obstacles_test.yaml" \
  auto_control_params_file:="$(pwd)/auto_control_test.yaml" \
  bev_params_file:="$(pwd)/bev_config_test.yaml" \
  line_detactor_params_file:="$(pwd)/line_detactor_test.yaml" \
  traffic_light_params_file:="$(pwd)/traffic_light_test.yaml" \
  input_mode:=slcan slcan_channel:=/dev/ttyACM0 slcan_bitrate:=500000 \
  can_controller_id:=112 preview_enabled:=true

# 실행 중 ON / OFF (다음 계획 주기 안에 반영, 기본 약 100ms + 계산 시간)
ros2 param set /auto_obstacles obstacles.avoidance.enabled true
ros2 param set /auto_obstacles obstacles.avoidance.enabled false

# 상세 상태/거절 이유
ros2 topic echo /auto/avoidance_preview/status
```

launch에서 `avoidance_test_enabled`를 생략하면 YAML의 `obstacles.avoidance.enabled`를
사용하며, YAML에도 없으면 false다. 명시한 launch 값이 YAML보다 우선한다.
`obstacles_enabled:=false`로 Depth 전체를 끈 실행에서는 이 기능을 켤 수 없다.
`manual_test:=true`는 기존과 같이 자동 액추에이터 출력을 막는다.
회피 테스트 자체는 `manual_test` 값과 관계없이 **화면 전용**이며 기존 중앙 경로 메시지,
조향/속도/신호등 정지 로직을 변경하지 않는다. `v`는 실제 목표 속도 명령이 아니다.

표시: 노란색=원래 중앙 경로, 흐린색=좌우 후보(녹색 계열은 통과 조건 만족),
청록색=선택 경로, 붉은 사각형=장애물 표면을 차량 크기/여유 거리만큼 확장한 영역.
`TEST LEFT/RIGHT`는 선택 방향, `CLEAR`는 중앙 경로 사용, `WAIT`는 관측/동기화 대기,
`BLOCKED`는 통과 가능한 후보가 없는 상태다. `v`는 권고 속도(m/s), `k`는 선택 경로의
최대 절대 곡률(1/m)이다. WAIT/BLOCKED에서는 선택 경로를 지우고 권고 속도를 0으로 둔다.

알고리즘은 전방으로 진행하는 중앙 경로에 5차 smoothstep 횡방향 이동/복귀 곡선을
적용한다. 중앙 경로에 걸리는 장애물 전체 구간을 회피하고, 후보마다 **모든 검출 군집**,
양쪽 관측 차선, BEV 좌우 범위, 최대 곡률을 확인한다. 후보가 없으면 반대 차선 밖으로
강제 우회하지 않는다. 이전 선택 방향을 유지하며 그 방향이 막히면 BLOCKED로 둔다.
장애물이 사라졌을 때는 새로운 Depth 관측에서 중앙 경로가 연속으로 비어 있는지
`clear_confirm_sec` 동안 확인한다. 입력 만료/동기화 실패는 이 확인 시간을 초기화한다.
이것은 깜빡임 완화이며 물체 ID 추적·차량 이동 보정에 의한 통과 판정은 아니다.

| `obstacles.avoidance.*` | 기본값 | 의미 |
|---|---:|---|
| `enabled` | false | 회피 경로 프리뷰 ON/OFF. 유일한 실시간 변경 항목 |
| `max_fps` | 10.0 | 별도 스레드의 계획 주기 상한. 차선 FPS 제한 아님 |
| `max_age_sec` / `max_sync_sec` | 0.25 / 0.06 | 계획 입력 만료 / RGB-Depth 시각 차이 |
| `vehicle_half_width_m` / `vehicle_half_length_m` | 0.15 / 0.25 | 앞차축 기준 차체의 좌우/전후 최대 거리 |
| `safety_margin_m` / `unknown_extent_m` | 0.04 / 0.04 | 차량 여유 / 관측하지 못한 물체 표면 여유 |
| `sample_step_m` | 0.025 | 후보 경로 샘플 간격 |
| `max_offset_m` / `offset_step_m` | 0.40 / 0.05 | 좌우 이동량 최대값 / 후보 증가 간격 |
| `transition_m` | 0.70 | 이동/복귀 각각의 최소 길이. 남는 관측 공간을 활용해 더 완만하게 생성 |
| `max_curvature_per_m` | 2.0 | 최대 곡률. 2이면 최소 회전반경 0.5m |
| `max_speed_mps` | 0.5 | 표시할 권고 속도 상한 |
| `lateral_acceleration_mps2` | 0.4 | 곡률에 따른 권고 속도 제한 |
| `deceleration_mps2` | 0.5 | 장애물까지 거리로 계산하는 정지 가능 속도 기준 |
| `clear_confirm_sec` | 1.0 | 중앙 경로가 다시 비었다고 판단하기 위한 연속 관측 시간 |

차체를 경로 접선 방향의 선분과 반폭의 원으로 감싸는 보수적인 캡슐 모델을 사용한다.
차선 여유는 차량 반폭 + 안전 여유 + 샘플 간 이동/회전 여유다. 앞차축에서 전후로
`vehicle_half_length_m`만큼 뻗은 선분으로 차량 길이를 반영하고, 장애물에는
`unknown_extent_m`를 추가한다. 붉은 사각형은 차량이 정면을 향할 때의 확장 영역이며,
실제 후보 검사는 각 위치의 회전 방향을 적용한다. 실제 차체 치수를 기준으로 설정한다.
차선 폭이 좁아 차량과 장애물이 함께 통과할 수 없으면 BLOCKED가 정상이다.

급격한 U자/비단조 경로, 양쪽 차선의 관측 공백, 3m BEV 안에서 이동·복귀 공간이 부족한
경우는 WAIT/BLOCKED로 제한한다. 정적인 국소 후보 생성이며 주행 중 이동 보정,
움직이는 장애물 예측, 차량 현재 조향과의 연속성 및 실제 저속 제어 연결은 후속 단계다.
이번 변경은 수동주행 화면에서 경로 기하를 검토하는 단계이며 빌드·테스트·실차 실행은 하지 않았다.

계획 주기가 차선 표시보다 느리므로 화면은 마지막 계획을 짧게 재사용한다.
현재 차선 화면과 계획의 촬영 시각 차이는 기본 최대 160ms(1/max_fps + max_sync_sec,
max_age_sec 이하)이며, 차선 입력/Depth 자체가 250ms를 넘으면 즉시 지운다.
이 표시 유지에는 차량 이동 보정이 없으므로 수동주행 화면 검토용으로만 사용한다.
