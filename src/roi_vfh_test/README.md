# roi_vfh_test — 공유 VFH+ 주행 정책

로컬 시뮬레이터에서 사용 중인 VFH+ 계획·차선 방향 기억·위험 셀 연속성 정책을
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
Start 전에는 제동 상태이며 설정을 변경할 수 있다. Start는 편집을 적용하고 ROI 잠금과
자동 출력 소유권 ACK를 기다린 뒤 실행한다. 출발 후 ROI/VFH 설정은 잠기며 Stop은 제동하고 잠금을 해제한다.
창 닫기·Stop은 재출발 의사를 취소하므로 다시 Start가 필요하다.

## Stop / Start 수동 전환

기본 launch는 `manual_drive_with_dynamics.launch.py`와 같은 8BitDo Joy 입력·수동 제어
프로필·VESC bridge·dynamics 구성 요소를 통합한다. 별도 터미널에서 그 launch를 함께 실행하지 않는다.
VESC와 SLCAN 포트는 각각 하나만 열고, 자동/수동 출력은 `manual_control/vfh_handover_node`가
실제 VESC 토픽에 단독 발행한다. VFH는 전용 `/autopilot03/handover/vfh/*` 입력에만 발행한다.
Joy와 dynamics는 launch 시작부터 준비하되 수동 명령은 Stop 전에는 차단한다.

- **Stop 버튼 또는 S/Space**: VFH 출력을 차단하고 기존 정지 제동 기준(기본 2A)을 적용한다.
  새로운 UART ERPM이 100 이하로 0.5초 유지되고, RT/LT/RB를 놓고 조향을 중앙에 둔
  최신 Joy 입력을 확인하면 MANUAL로 전환한다. 이후 기존 RT/LT/조향/RB 조작을 사용한다.
- **Start**: 수동 출력을 즉시 차단하고 같은 정지·중립 확인을 거쳐 AUTO로 전환한다.
  요청별 ACK와 10Hz 상태 수명을 확인한 뒤 최신 입력으로 VFH를 실행한다. 전환 중 조작을
  놓지 않거나 ERPM/Joy가 끊기면 제동을 유지한다. 이전 duty/수동 입력은 재생하지 않는다.
- **경로 없음·센서 유실 등 자동 정지**: 수동 모드로 전환하지 않는다. 기존 제동·입력 복구 정책을 유지한다.
- 창 닫기는 전체 launch/프로세스 종료 경로이며 수동 전환 버튼이 아니다. X 버튼 전환은
  기존 auto_drive 전용이며 이 VFH 전환 노드에서는 사용하지 않는다.

요청한 기본 옵션은 `vehicle_namespace:=autopilot03`, `vesc_port:=/dev/ttyTHS1`,
`controller_name_contains:=8BitDo`, `input_mode:=slcan`, `slcan_channel:=/dev/ttyACM0`,
`slcan_bitrate:=500000`, `can_controller_id:=112`다. VFH 속도와 모드 전환 정지 확인은
항상 VESC bridge의 UART ERPM을 사용한다. `roi_vfh_dynamics`가 UART ERPM을 환산해
`/roi_vfh_test/measured_speed`를 단독 발행한다. CAN 계측은 `roi_vfh_can_dynamics` 한
인스턴스가 별도로 수행하고 `/autopilot03/vehicle/dynamics/*`로 발행한다. CAN이 끊겨도
정상 UART 속도를 가리지 않으며, UART 속도가 만료되면 기존 제동 정책을 유지한다.
UART 속도 환산만 필요하면 `input_mode:=ros_topic`으로 CAN 계측을 끈다. 수동 전환 없이 기존 직접 출력으로
실행하려면 `manual_handover_enabled:=false`를 지정한다. `drive_enabled:=false`에서는
수동 전환 노드/Joy를 실행하지 않는다. 수동 서보·Duty 설정은 기존 manual YAML을 사용한다.

수정 적용에는 위 `colcon build --packages-up-to roi_vfh_test`를 사용한다. 신규
`manual_control/vfh_handover_node`도 함께 설치되어야 한다. 이번 변경에서 ROS 실행·실차 테스트는 수행하지 않았다.

## 하나의 정책 코어

- `include/roi_vfh_test/planner.hpp`: 두 실행 환경이 공유하는 VFH+·차체 충돌 검사·후보 점수.
- `include/roi_vfh_test/policy_state.hpp`: 두 실행 환경이 공유하는 현재 차선 채택/선택 가능한 방향 기억.
- `include/roi_vfh_test/hazard_filter.hpp`: 현재 관측의 작은 위험 셀 연속성 확인.
- `src/preview_node.cpp`: ROS 촬영 시각 정렬·수신 유효성·GUI·구동 어댑터.
- `auto_control/tools/vfh_sim/native_planner.cpp`: 같은 코어를 호출하는 시뮬레이터 어댑터.

시뮬레이터의 이전 전용 planner 헤더는 공유 헤더로 연결하는 호환용 파일이다.
실차와 시뮬레이터를 각각 수정한 다른 계획기가 다시 생기지 않도록 했다.

## 주행 정책

1. 현재 유효 중앙선을 추종하는 차체·패딩 궤적에
   위험 셀이 겹치지 않으면 기존 Stanley 기반 차선 추종/복귀를 사용한다.
2. 차선 추종이 장애물에 막히거나 현재 차선이 없으면 저장된 방향/전방 선호로 VFH+ 후보를 검사한다.
   장애물 영향은 `horizon_m` 범위에서 검사하며 속도에 따른 정지 거리로 회피 개시 범위를 줄이지 않는다.
3. VFH+의 primary → binary hysteresis → 회전/차폭 masked → Ackermann 후보 검증 흐름과
   추가 곡선을 사용한다. 히스토그램이 전부 막혀도 추가 곡선을 검사한 뒤 정지 여부를 결정한다.
4. 모든 후보는 동일한 회전 차체 직사각형과 현재 위험 격자로 검사한다.
   차체와 패딩은 앞차축 기준이다. 격자 밖과 미관측 회색 셀은 통과 가능으로 가정한다.
5. 가장 긴 후보의 진행량을 우선하고 `avoidance_length_slack_m` 이내 후보에서
   전방 격자 장애물 여유·차선 거리/접선·조향 연속성 점수를 비교한다.
   현재 차선/저장된 방향 쪽은 선호이며 경로를 차선 밖 거리로 잘라내지 않는다.
6. 유효 경로가 없으면 목표 속도 0이다. 유효 경로의 길이·곡률·감속도·반응 지연으로 속도를 제한한다.

## 현재 차선과 방향 기억

현재 유효 중앙선은 첫 관측부터 사용한다. 과거 차선과의 위치/각도 일관성이나
0.8초 연속 검증으로 중앙선을 버리지 않는다. 최소 8점/40cm 및 15cm 점 간격 절단도
제거했다. 서로 다른 유한 좌표 2점 이상을 현재 기준으로 사용한다.

최신 terrain과 차선의 촬영 시각 차이를 60ms로 제한하지 않는다. 이동 이력에 차선
촬영 자세가 있으면 terrain 촬영 자세로 보정하고, 없으면 현재 관측 좌표를 사용한다.
차선 자체의 실제 수신/촬영 수명·프레임/좌표 유효성 검사는 유지한다.

현재 차선 추종의 차체 궤적이 비어 있으면 Stanley로 추종·복귀한다. 짧은 현재 차선의
끝을 넘어서는 예측은 마지막 접선으로 이어서 수행하고, 전체 예측 궤적의 장애물 충돌을 검사한다.
장애물이 궤적을 막으면 VFH로 차선 밖 회피를 허용하며, 차선 복귀 가중치를 기본
이미지에서 지정한 기본값 1.2를 사용한다. 차선 복귀는 선호이며 장애물 충돌 검사를 우회하지 않는다.

`lane_memory_enabled`는 기본 `1.0`(ON), `0.0`은 OFF다. Stop → VFH parameters
첫 페이지에서 변경하고 Apply → Start로 적용한다. YAML 또는 Stop 상태의
`ros2 param set /roi_vfh_test lane_memory_enabled 0.0`으로도 변경할 수 있다.

ON은 마지막 관측의 **방향각 하나만** 저장한다. 중앙선 전체 형상이나 횡위치는 저장하지 않는다.
차선이 안 보이면 차량 회전량을 반영한 그 방향을 VFH 선호 방향으로 사용한다.
방향만으로는 미관측 중 중앙선의 정확한 횡위치를 알 수 없다. 중앙선이 다시 보이면
현재 형상을 즉시 사용해 중앙 복귀를 유도한다. OFF에서 차선도 없으면 전방 선호로 VFH를 사용한다.
파라미터 변경·Stop/Start·이동 이력 초기화는 방향 기억을 지우며 ON 재설정은 이전 기억을 복원하지 않는다.

## 현재 장애물 관측

장애물 위치의 월드 좌표 저장·사각지대 복원·기억 표본 충돌 검사를 제거했다.
후보 차체 검사는 현재 terrain의 위험 셀만 사용한다. `obstacle_memory_enabled` 설정도 제거했다.
차량 크기·패딩·현재 위험 셀 검사는 유지한다.

작은 hazard는 현재 원본 관측이 있는 셀만 대상으로 8방향 연결 성분/움직임 정렬을 사용해
기본 4개 depth 관측 연속성을 확인한다. 8셀 초과 덩어리는 즉시 반영한다.
미확정 셀은 회색이고 현재 정책에서 통과 가능이다. 자유 영역 지도 누적은 하지 않는다.

## 선택한 기본값과 GUI

| ROS 파라미터 | 기본값 |
|---|---:|
| `nominal_speed_mps` | 1.2 |
| `horizon_m` / `reference_distance_m` | 1.2 / 1.0 |
| `safety_margin_m` | 0.02 |
| `sample_step_m` | 0.03 |
| `lane_lookahead_m` | 0.05 |
| `avoidance_curve_samples` | 17 |
| `avoidance_length_slack_m` | 0.40 |
| `avoidance_lane_weight` | 1.2 |
| `avoidance_steering_weight` | 0.2 |
| `avoidance_clearance_weight` | 0.7 |
| `deceleration_mps2` / `response_delay_sec` | 0.4 / 0.10 |

위 항목과 차선 기억 ON/OFF·히스토그램/위험 셀 연속성 항목을 총 22개 숫자 입력, 2페이지로 표시한다.
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

## 시뮬레이터와 공유하는 속도 정책

`speed_policy.hpp`의 `TargetSpeedPolicy`를 실차와 로컬 C++ worker가 함께 호출한다.
장애물 감속 유지·목표 속도 회복 계산은 기존 식을 그대로 공통 함수로 옮겼다.
시뮬레이터는 이 패키지의 preview/reference YAML에서 기본값과 고정 제어 설정을 읽는다.
RGB/Depth/계획/구동 시계를 분리하고, 서로 다른 촬영 자세는 공통 `reproject`로 정렬한다.
센서·통신은 정상, 위치는 정확하다고 가정하며 차량의 종방향 응답은 기존 가감속 모델이다.
실제 VESC Duty·브레이크 응답이나 신경망/Depth 잡음 검증은 포함하지 않는다.

## auto_control과 FPS 통일

VFH launch는 선택한 참조 YAML의 속도를 사용한다. 일반 auto_control 기본값과 맞춘 값은 다음과 같다.

| 경로 | 설정값 | 출처 |
|---|---:|---|
| RGB / 차선용 BEV | 최대 80FPS | camera `sensor_fps`, `publish_fps`; 양수 BEV `publish_max_fps`가 있으면 함께 제한 |
| 장애물 Depth / terrain | 요청 40FPS | obstacles `obstacles.camera.fps` |
| VFH 계획 | 80Hz | obstacles `obstacles.avoidance.max_fps` |
| ROI / VFH 프리뷰 | 최대 30FPS | BEV `preview_max_fps` |
| 제어 / IMU / VESC 속도 | 100Hz / 요청 120Hz / 80Hz | 기존 제어·VESC 설정; VFH 전용 `imu_rate_hz` launch 인자 |

RGB 수신을 depth 처리 스레드에서 분리했고, 차선 BEV 발행도 프리뷰 FPS 제한에서 분리했다.
새 RGB 촬영본당 한 번만 차선 BEV를 발행한다. 프리뷰가 꺼져 있거나 30FPS여도
차선 입력을 같은 속도로 낮추지 않는다. VFH 계획 빈도를 올려도 같은 terrain 촬영본을
여러 프레임 관측한 것으로 세지는 않는다. 일반 roi_lidar 단독 실행의 기본 FPS는 유지한다.
실제 도달 FPS는 장치·전송·추론·계획 처리 성능에 따르며, 이번 변경에서는 실측하지 않았다.

## 반복 입력·표시 비용 최적화

일반 auto_control의 회피 노드와 달리 VFH는 고정 80Hz 계획 타이머와
100Hz 구동 감시를 이미 분리했고, 같은 차선 + 새 terrain의 재계획도 허용한다.
ROI 광선은 영상 기하/자세 또는 설정이 바뀔 때만 재생성한다. 차선 BEV 입력은
표시용 영상이 아니라 인식 입력이므로 기존 80FPS 경로를 유지한다.

추가로 다음 중복을 제거했다.

- 같은 선택 terrain은 hazard 결과를 재사용한다. 새 terrain에서는 기존대로
  중간 관측까지 시간순으로 처리해 빈 관측이 약한 hazard 연속성을 끊도록 한다.
- Start/Stop, 파라미터 변경, 입력 대기 진입, odometry 세대 변경 시 캐시를 버린다.
  같은 terrain도 속도·IMU를 통한 촬영 자세 정렬 가능 여부가 바뀌면 다시 계산한다.
- 필터 평균 시간은 실제 필터 실행만 집계하며, 같은 입력을 재사용할 때는 격자를
  다시 복사하지 않는다. 현재 선택 경로의 기억 장애물 표시도 중간 tick에서 지우지 않는다.
- 재계획 여부는 ROS 메시지 포인터 대신 terrain/차선 촬영 시각 쌍으로 판정한다.
  새 terrain, 새 차선, 차선 유실/복구, odometry 세대 변경은 계속 재계획한다.
- 입력 대기에서는 최초 진입에 즉시 정지 명령을 보내고, 이후 기존 100Hz
  actuator 타이머가 제동을 유지한다. 계획 루프에서 같은 제동 명령을 추가 반복하지 않는다.
- 별도 BEV 창과 BEV 구독자가 없으면 ROI가 그 화면용 투영 맵을 만들지 않는다.
  차선 인식용 BEV와 파라미터/ROI 선택 창은 유지한다.

이번 추가 변경도 빌드·테스트·실차 실행 없이 소스와 Git diff만 검토했다.

## 실차 어댑터와 입력 만료

VESC bridge 자동 실행과 namespace/명시 토픽 override, 속도 환산·서보/PID/FF 보정을 유지한다.
계획은 `auto_control` 회피 설정과 같은 기본 80Hz, 구동·정지 감시는 100Hz다. 기존 `DriveController`를 사용한다.
정지 시 Duty 0 상태와 최대 2A 제동, 조향 유지 정책을 유지한다.
실제 모터 명령은 브레이크와 Duty가 서로 덮이지 않도록 기존의 한 모드씩 발행하는 절차를 따른다.

- lane/terrain 촬영 최대 수명 0.30초, VESC 속도 0.15초, IMU 0.25초 검사 적용.
- 새 입력을 기다릴 때 아직 수명이 남은 직전 경로 유지. 만료되면 제동.
- 일시적 입력 문제는 0.50초 연속 정상 입력과 유효 경로 후 자동 재출발. Stop은 이를 취소.
- VESC 단절, 다른 구동 발행자, 좌표 불일치, 잠금 상실 등의 기존 정지 검사 유지.
- 실측 입력 나이 + 계획 주기보다 `working.delay`가 작아지지 않는 기존 하한 유지.

IMU·VESC 속도·terrain·차선 구독은 전용 single-threaded executor에서 처리한다.
VFH 계산/HighGUI는 원래 main 스레드에 남기고, 계산 직전과 구동 감시 시 mutex로 보호한
입력/이동 이력 사본을 가져온다. 계산·화면 갱신이 수신 콜백을 밀어 이동 이력을 끊거나
DDS terrain 대기열을 쌓지 않도록 했다. terrain DDS 대기열은 최신 1개이고,
이미 수신한 최근 16개 관측은 hazard 연속성 확인용으로 보존한다.

계획에는 촬영 자세를 정렬할 수 있는 최신 terrain을 먼저 선택한다. 현재 유효 차선은
촬영 시각 차이로 제외하지 않는다. 현재 차선이 없으면 저장된 방향 또는 전방 선호를 사용한다.
정렬을 기다리는 동안 직전 경로를 쓰는 경우에도 원래 촬영 수명 검사를 통과해야 한다.
실제 센서 단절·지형 만료·이동 이력 초기화 시 제동은 유지하며, IMU 외삽 범위도 바꾸지 않았다.
상태 로그에는 terrain/IMU/속도 입력 나이와 이동 이력 세대를 함께 기록한다.

## 출력과 로그

`~/path`, `~/target_speed`, `~/steering_angle_rad`, `~/status`, `~/preview`와
최종 Duty/브레이크/서보 진단 출력을 유지한다. 기본 구동 모드에서는 기존 VESC 토픽으로 명령을 보낸다.
상태/HUD에 차선 입력 출처, 방향 기억 유무·나이, 방향 기억 ON/OFF, 후보 종료/최장 길이를 표시한다.
CSV에는 `lane_trusted`, `lane_memory`, `lane_memory_age_sec`를 기록한다.
장애물 기억 열 `remembered_blind`, `remembered_world`는 제거했다. 삭제된 이탈 한도/격자 경계를 정지 이유로 표시하지 않는다.

## 검증 상태

사용자 요청대로 이번 작업에서는 **빌드·테스트·시뮬레이터 재주행·실차 실행을 하지 않았다.**
새 API와 정책에 맞춰 기존 회귀 테스트 소스를 갱신하고 차선 기억 회귀 소스를 갱신하고 삭제한 장애물 기억 회귀는 제거했지만 실행하지 않았다.
소스 검토와 Git diff의 공백 오류 확인만 수행했다.

## VFH IMU 요청 주기와 허용 수명

VFH 실행의 기본 IMU 요청은 120Hz이고 수신·촬영 수명 제한은 0.25초다.
RGB/Depth 주기와 일반 auto_control 카메라 설정은 바꾸지 않는다.
실제 IMU 출력 주기는 장치가 지원하는 주기에 따라 요청값과 다를 수 있다.

```bash
ros2 launch roi_vfh_test preview.launch.py imu_rate_hz:=120
```

허용 수명은 VFH `params_file`의 `roi_vfh_test.ros__parameters.imu_timeout_sec`에서
변경하며 재실행해야 한다. 새 IMU 표본의 원래 촬영 시각은 계속 검사한다.
이번 변경은 사용자 요청에 따라 빌드·테스트를 실행하지 않았다.

중앙선 추종이 약하게 보이는 코드 경로와 관측의 한계는 [차선 추종 분석](LANE_FOLLOWING_ANALYSIS_KO.md)에 정리했다.
