# roi_vfh_test — 공유 VFH+ 주행 정책

로컬 시뮬레이터에서 사용 중인 VFH+ 계획·차선 신뢰도/기억·근거리 장애물 기억 정책을
ROS 입력과 VESC 제어에 연결한다. 별도의 통로 분류, 입구/미션 판정, 차선 이탈 한도는 없다.
시뮬레이터 마킹·트랙·장애물 원본 객체 목록은 이 패키지의 입력이 아니다.

## 실행

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to roi_vfh_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roi_vfh_test preview.launch.py
```

위 명령은 사용 방법이며 이번 이식 작업에서는 빌드·테스트·실차 실행을 하지 않았다.
기본 launch는 VESC bridge, ROI OAK-D 입력, 차선 인식, 속도 환산, VFH를 함께 실행한다.
`vehicle_bringup vesc_only.launch.py`를 별도로 중복 실행하지 않는다.
이미 bridge를 실행 중이면 `vesc_bridge_enabled:=false`를 지정한다.
구동 없이 확인하려면 `drive_enabled:=false`를 지정한다.

ROI 설정창과 VFH 프리뷰가 표시된다. 별도 BEV 창은 열지 않는다.
Start 전에는 제동 상태이며 설정을 변경할 수 있다. Start는 편집을 적용하고 ROI 잠금
ACK를 기다린 뒤 실행한다. 출발 후 ROI/VFH 설정은 잠기며 Stop은 제동하고 잠금을 해제한다.
창 닫기·Stop은 재출발 의사를 취소하므로 다시 Start가 필요하다.

## 하나의 정책 코어

- `include/roi_vfh_test/planner.hpp`: 두 실행 환경이 공유하는 VFH+·차체 충돌 검사·후보 점수.
- `include/roi_vfh_test/policy_state.hpp`: 두 실행 환경이 공유하는 차선 신뢰도/전체 형상 기억과 장애물 기억.
- `include/roi_vfh_test/hazard_filter.hpp`: 현재 관측의 작은 위험 셀 연속성 확인.
- `src/preview_node.cpp`: ROS 촬영 시각 정렬·수신 유효성·GUI·구동 어댑터.
- `auto_control/tools/vfh_sim/native_planner.cpp`: 같은 코어를 호출하는 시뮬레이터 어댑터.

시뮬레이터의 이전 전용 planner 헤더는 공유 헤더로 연결하는 호환용 파일이다.
실차와 시뮬레이터를 각각 수정한 다른 계획기가 다시 생기지 않도록 했다.

## 주행 정책

1. 현재 차선이 연속 관측 신뢰도 검사를 통과하고, 그 차선을 추종하는 차체·패딩 궤적에
   위험 셀이 겹치지 않으면 기존 Stanley 기반 차선 추종/복귀를 사용한다.
2. 차선 추종이 장애물에 막히거나 차선이 없거나 아직 신뢰되지 않으면 VFH+ 후보를 검사한다.
   장애물 영향은 `horizon_m` 범위에서 검사하며 속도에 따른 정지 거리로 회피 개시 범위를 줄이지 않는다.
3. VFH+의 primary → binary hysteresis → 회전/차폭 masked → Ackermann 후보 검증 흐름과
   추가 곡선을 사용한다. 히스토그램이 전부 막혀도 추가 곡선을 검사한 뒤 정지 여부를 결정한다.
4. 모든 후보는 동일한 회전 차체 직사각형과 위험 격자/기억 표본으로 검사한다.
   차체와 패딩은 앞차축 기준이다. 격자 밖과 미관측 회색 셀은 통과 가능으로 가정한다.
5. 가장 긴 후보의 진행량을 우선하고 `avoidance_length_slack_m` 이내 후보에서
   전방 격자 장애물 여유·차선 거리/접선·조향 연속성 점수를 비교한다.
   기억 차선 쪽은 선호이며 경로를 차선 밖 거리로 잘라내지 않는다.
6. 유효 경로가 없으면 목표 속도 0이다. 유효 경로의 길이·곡률·감속도·반응 지연으로 속도를 제한한다.

## 차선 신뢰도와 기억

- 기본 0.8초 동안 연속 관측, 관측 경로 길이 0.4m 이상, 위치 일관성 0.15m,
  접선 일관성 12도 조건을 적용한다. 관측 간격이 0.15초를 넘으면 확인을 다시 시작한다.
- 같은 카메라 촬영 결과를 여러 depth 프레임과 재사용해도 확인 시간이 늘지 않는다.
- 신뢰한 중앙선 **전체 형상**을 차량 이동 좌표로 보존한다. 마지막 방향 하나만 저장하지 않는다.
- 차선이 없어도 VFH로 진행하고, 새 차선이 신뢰될 때까지 기억 형상 쪽 복귀를 선호한다.
  기억 끝을 지나면 마지막 접선 방향으로 연장한다. 차선을 처음부터 한 번도 신뢰하지 못했다면
  차량 전방을 선호하며 차선 복귀 비용은 0으로 둔다.
- 신뢰된 현재 차선이 다시 들어오고 그 추종 궤적이 충돌 없이 열리면 직접 차선 추종으로 복귀한다.
- 관측 좌표계가 연속인 주행 중 차선 기억에는 시간 만료를 두지 않는다.
  Stop/Start·파라미터 변경·이동 이력 초기화 시에는 현재 세션의 기억을 초기화한다.
  입력 일시 정지 중에도 이동 좌표가 연속이면 기억은 유지하며, 이동 이력이 재설정되면 재출발 전 초기화한다.

## 장애물 기억 — 현재 시뮬레이터와 동일

현재 프레임의 확정 위험 셀을 차량 이동 좌표에 보존한다. 현재 전방 격자 안에서는 현재
관측을 사용하고, 가까운 후방/측면 미관측 공간에 들어간 표본은 차체 검사에 사용한다.
완전히 지나간 뒤쪽 표본 또는 기억 범위 밖 표본은 제거한다.
범위는 차체 크기와 격자 크기로 계산하며 별도의 통로 상태나 시간 감쇠를 추가하지 않는다.

사용자 선택대로 **알려진 기억 표본/확대 차체 겹침 정지 증상을 포함해 그대로 이식**했다.
기억 표본을 현재 자유 관측으로 지우거나, 패딩을 자동 축소하거나, 이미 겹친 표본을
무시하는 변경은 하지 않았다. 시뮬레이터의 `memoryTime`/`memoryDistance`는 이 네이티브
기억 로직의 설정이 아니며 이 ROS 패키지에도 그 이름의 조절 기능을 추가하지 않았다.

작은 hazard는 현재 원본 관측이 있는 셀만 대상으로 8방향 연결 성분/움직임 정렬을 사용해
기본 4개 depth 관측 연속성을 확인한다. 8셀 초과 덩어리는 즉시 반영한다.
미확정 셀은 회색이고 현재 정책에서 통과 가능이다. 자유 영역 지도 누적은 하지 않는다.

## 선택한 기본값과 GUI

| ROS 파라미터 | 기본값 |
|---|---:|
| `nominal_speed_mps` | 1.0 |
| `horizon_m` / `reference_distance_m` | 1.2 / 1.0 |
| `safety_margin_m` | 0.02 |
| `sample_step_m` | 0.03 |
| `lane_lookahead_m` | 0.05 |
| `avoidance_curve_samples` | 17 |
| `avoidance_length_slack_m` | 0.40 |
| `avoidance_lane_weight` | 0.8 |
| `avoidance_steering_weight` | 0.1 |
| `avoidance_clearance_weight` | 1.0 |
| `lane_trust_duration_sec` | 0.8 |
| `lane_trust_min_length_m` | 0.4 |
| `lane_trust_max_offset_m` | 0.15 |
| `lane_trust_max_heading_deg` | 12.0 |
| `deceleration_mps2` / `response_delay_sec` | 0.4 / 0.10 |

위 항목과 히스토그램/위험 셀 연속성 항목을 총 25개 숫자 입력, 3페이지로 표시한다.
출발 후 편집은 잠기고 페이지 조회는 가능하다. `lane_departure_m`은 제거했다.
속도/감속도의 기존 차량 설정 상한은 계속 적용한다.

차체는 폭 0.30m, 축간거리 0.33m, 앞차축→앞끝 0.05m, 앞차축→뒤끝 0.45m다.
기존 `vehicle_half_length_m=0.25`를 앞차축 뒤쪽 길이로 사용하지 않는다.
VFH YAML의 lookahead·geometry·margin·step 값은 다른 참고 YAML로 덮어쓰지 않는다.

## ROI·센서·연산 설정

VFH 전용 `config/reference/roi_lidar_test.yaml`의 terrain 셀은 **0.025m**다.
폭 1.5m×전방 4m, 차선 BEV 1.2m×3m는 유지한다. 7cm 대비 셀 수는 약 8배이며,
지면/장애물 최소 표본 수·깊이 필터·고립점 제거 설정은 변경하지 않았다.
일반 `roi_lidar` 실행 YAML의 7cm 기본값에는 영향을 주지 않는다.

시작 지면 평면 재사용(`ground.use_startup_plane=true`), 프레임별 RANSAC 기본 OFF,
별도 장애물 군집 OFF를 유지한다. 별도 카메라 노드 없이 ROI가 RGB·depth·IMU를 공유하며,
초기 지면 자세 측정 프리뷰는 기존 방식으로 표시 후 닫힌다.
차량 좌표 IMU와 VESC 속도를 적분한 앞차축 이동 이력으로 촬영 시각을 정렬한다.
ROS에서는 실제 센서 수신·추론 지연이 존재하므로 시뮬레이터의 이상적인 자세 가정과 다르다.

기존 평균 연산 시간 HUD(ROI host/scan/grid, VFH hazard/plan)를 유지한다.
새 계획 코어는 차체/점수에서 접근한 셀의 차선 거리만 계산해 재사용하고,
최종 선택 경로만 출력으로 복사한다. 이번 이식으로 성능을 실측했다고 주장하지 않는다.

## auto_control과 FPS 통일

VFH launch는 선택한 참조 YAML의 속도를 사용한다. 일반 auto_control 기본값과 맞춘 값은 다음과 같다.

| 경로 | 설정값 | 출처 |
|---|---:|---|
| RGB / 차선용 BEV | 최대 80FPS | camera `sensor_fps`, `publish_fps`; 양수 BEV `publish_max_fps`가 있으면 함께 제한 |
| 장애물 Depth / terrain | 요청 40FPS | obstacles `obstacles.camera.fps` |
| VFH 계획 | 80Hz | obstacles `obstacles.avoidance.max_fps` |
| ROI / VFH 프리뷰 | 최대 30FPS | BEV `preview_max_fps` |
| 제어 / IMU / VESC 속도 | 100Hz / 400Hz / 80Hz | 기존 auto_control / camera / VESC 설정 |

RGB 수신을 depth 처리 스레드에서 분리했고, 차선 BEV 발행도 프리뷰 FPS 제한에서 분리했다.
새 RGB 촬영본당 한 번만 차선 BEV를 발행한다. 프리뷰가 꺼져 있거나 30FPS여도
차선 입력을 같은 속도로 낮추지 않는다. VFH 계획 빈도를 올려도 같은 terrain 촬영본을
여러 프레임 관측한 것으로 세지는 않는다. 일반 roi_lidar 단독 실행의 기본 FPS는 유지한다.
실제 도달 FPS는 장치·전송·추론·계획 처리 성능에 따르며, 이번 변경에서는 실측하지 않았다.

## 실차 어댑터와 입력 만료

VESC bridge 자동 실행과 namespace/명시 토픽 override, 속도 환산·서보/PID/FF 보정을 유지한다.
계획은 `auto_control` 회피 설정과 같은 기본 80Hz, 구동·정지 감시는 100Hz다. 기존 `DriveController`를 사용한다.
정지 시 Duty 0 상태와 최대 2A 제동, 조향 유지 정책을 유지한다.
실제 모터 명령은 브레이크와 Duty가 서로 덮이지 않도록 기존의 한 모드씩 발행하는 절차를 따른다.

- lane/terrain 촬영 최대 수명 0.30초, VESC 속도 0.15초, IMU 0.10초 검사 유지.
- 새 입력을 기다릴 때 아직 수명이 남은 직전 경로 유지. 만료되면 제동.
- 일시적 입력 문제는 0.50초 연속 정상 입력과 유효 경로 후 자동 재출발. Stop은 이를 취소.
- VESC 단절, 다른 구동 발행자, 좌표 불일치, 잠금 상실 등의 기존 정지 검사 유지.
- 실측 입력 나이 + 계획 주기보다 `working.delay`가 작아지지 않는 기존 하한 유지.

## 출력과 로그

`~/path`, `~/target_speed`, `~/steering_angle_rad`, `~/status`, `~/preview`와
최종 Duty/브레이크/서보 진단 출력을 유지한다. 기본 구동 모드에서는 기존 VESC 토픽으로 명령을 보낸다.
상태/HUD에 차선 신뢰도, 기억 유무·나이, 근처 기억 위험 수, 후보 종료/최장 길이를 표시한다.
CSV에는 기존 주행/위험 셀 기록에 `remembered_blind`, `remembered_world`, `lane_trusted`,
`lane_memory`, `lane_memory_age_sec`를 추가했다. 삭제된 이탈 한도/격자 경계를 정지 이유로 표시하지 않는다.

## 검증 상태

사용자 요청대로 이번 작업에서는 **빌드·테스트·시뮬레이터 재주행·실차 실행을 하지 않았다.**
새 API와 정책에 맞춰 기존 회귀 테스트 소스를 갱신하고 차선/장애물 기억 회귀 소스를 추가했지만 실행하지 않았다.
소스 검토와 Git diff의 공백 오류 확인만 수행했다.
