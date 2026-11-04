# longitudinal_autotune

4 m × 4 m 트랙, 정지선 두 개를 위한 독립 종방향 시험 패키지입니다. 기존 패키지의 소스·설정은 수정하지 않습니다. 설치된 RGB 차선/정지선 인식기와 `auto_control` 실행 파일을 재사용하며, 별도 실행·토픽·설정·로그·실험 상태를 갖습니다. 장애물/IR/터널/신호등 추론은 실행하지 않습니다.

현재 구현 범위는 **실차 반복 시험, 제한된 파라미터 탐색, 정차 평가, 최적 후보 저장, 오프라인 유효 종방향 모델 추정**입니다. 최적값은 시험한 속도·노면·배터리·차량에서의 최선 후보이며, 전역 최적해나 검증된 전체 차량 시뮬레이터라는 의미는 아닙니다.

## 시험 구조

- 시계 방향 `cw`, 반시계 방향 `ccw` 각각 기본 2바퀴, 두 정지선에서 매번 2초 정차 후 출발합니다. 후보 하나당 **8회 정차**를 평가합니다.
- 각 방향 블록의 첫 정차는 기준점을 잡는 **추가 정차**로 점수에서 제외합니다. 이후 S1 → S0 → S1 → S0 네 구간을 평가합니다. S0는 첫 기준 정지선입니다. 두 방향 모두 같은 물리 정지선을 첫 기준으로 만나도록 출발 위치를 정하면 선별 비교가 가능합니다. 절대 지도 좌표나 정지선 고유 ID를 인식하는 기능은 없습니다.
- 도면처럼 서로 반대편 직선에 정지선이 있는 트랙을 가정합니다. 정차 사이의 IMU 누적 회전이 방향별 ±π 부근인지 확인하여 반대 방향 주행이나 정지선 누락을 거릅니다. 트랙을 다르게 구성하면 해당 정책값을 바꾼 새 캠페인이 필요합니다.
- 바깥·안쪽 코너 반지름을 같다고 가정하지 않습니다. 실제 인식한 **중앙 경로 곡률**을 기존 제어기에 전달합니다. 외곽 4 m 치수로 경로 길이를 16 m라고 계산하지 않습니다.
- 두 방향 전환에는 차량 방향을 사람이 바꾸고 launch를 다시 시작해야 합니다. 좁은 트랙에서 자동 U턴·후진은 하지 않습니다. 방향을 바꿀 때 동일한 `campaign_dir`를 사용합니다.

정지선만으로 기존 신호 정지 로직을 시험하기 위해 내부 어댑터가 전용 `/longitudinal_autotune/stop_request`에 `TrafficLightState` RED/GREEN을 발행합니다. RED는 정지 요청, GREEN은 정차 유지 완료 후 해제입니다. **신호등을 검출했다는 기록으로 해석하면 안 됩니다.** 실제 신호등 인식 모델·공용 신호 토픽은 사용하지 않습니다.

## 평가·탐색 정책

설정: `config/experiment.yaml`.

| 항목 | 기본 정책 |
|---|---|
| 직선 최고 속도 | 0.8 m/s, 고정 |
| 코너 속도 | 중앙 경로에 대한 `sqrt(0.5 / abs(curvature))`, 기존 제어기의 예측 감속·최저 속도 적용 |
| 정지 범위 | 범퍼 기준 정지선 전방 0–25 cm, 점수 기준점 12 cm |
| 정차 확인 | ERPM 환산 속도 ≤ 0.03 m/s를 0.3초 확인 |
| 정차 유지 | 2초, 적분 이동량 ≤ 1.5 cm; 속도 > 0.06 m/s 또는 양의 duty가 나오면 중단 |
| 부드러움 | 속도에 고정 80 ms 필터 → 가속도·jerk; jerk RMS, 최대 감속, 구동/제동 반복 전환 평가 |
| 코너·주행 | 곡률 속도 상한 초과량, 목표 속도 추종 오차, 횡오차 최대/RMS/20 cm 초과 시간·면적·회복 횟수, 구간 시간 기록 |
| 횡오차 평가 | 20 cm 초과는 경고·감점 후 계속 주행; 별도 40 cm 초과 시 보호 중단 |
| 데이터 | 차선/ERPM/IMU/제어 명령 수명, 중복·역순 차선 timestamp, 로그 누락 검사 |

정지 유지 중에는 **시험 대상 제어기의 명령을 그대로 전달**하여 꿈틀거림을 숨기지 않습니다. 데이터 이상·별도 보호 한도 위반·정차 유지 중 큰 움직임을 발견하면 시험 게이트가 제동으로 전환합니다. 횡오차 평가 기준 20 cm 초과는 감점과 기록에만 사용하며 즉시 정지하지 않습니다. 정상 주행은 기존의 duty 감소 → 감속 반응 확인 → 부족할 때 전류 제동 절차를 그대로 사용합니다. 보호 중단의 제동은 점수 대상 정상 감속과 구분합니다.

정지 거리와 유지 이동량은 BEV 거리 및 ERPM 적분에 의한 **추정값**입니다. 카메라 기하, 앞차축→범퍼 거리, 타이어/구동비가 틀리거나 바퀴가 미끄러지면 실제 위치 오차와 다릅니다. 이 설정들은 탐색 중 바꾸지 않습니다. 실측 없이 절대 정지 정확도를 인증하거나 질량·마찰계수를 개별 식별했다고 할 수 없습니다.

바꾸는 파라미터는 다음 6개뿐입니다.

1. `longitudinal_planning_deceleration_mps2`
2. `duty_rise_rate_per_sec`
3. `duty_fall_rate_per_sec`
4. `brake_current_rise_amps_per_sec`
5. `brake_current_fall_amps_per_sec`
6. `corner_planning_acceleration_mps2`

기준 설정을 먼저 양방향에서 검증한 뒤, 현재 최선 설정의 한 항목을 제한 범위 내에서 ± 변경하는 좌표 탐색을 합니다. 개선이 없으면 변경 폭을 줄입니다. 최대 20개 후보입니다. 목적함수는 정지 위치·jerk·유지 이동·구동/제동 전환·속도 추종·주행 시간·코너 과속·경로 추종의 가중합이며, 양방향 8개 점수의 평균 + 0.3×표준편차로 비교합니다. 기본 2% 이상 개선된 후보만 승격합니다. 통계적 유의성이나 수렴을 보장하는 기준은 아니며 반복 횟수를 늘려 재검증할 수 있습니다.

후보 교체는 방향 블록이 끝나고 정차가 확인된 상태에서만 수행합니다. 출력 게이트가 정지 상태를 유지하면서 기존 `auto_control` 프로세스를 종료하고 새 YAML로 재시작합니다. 이전 프로세스가 살아 있는 동안 다음 프로세스를 시작하지 않습니다. 다음 방향의 기록이 필요한 경우 정지한 채 기다립니다. 양방향 검증이 끝나면 현재 방향의 다음 후보 블록은 자동으로 진행할 수 있습니다.

센서·차선·통신·로그·운전자 중단은 불완전 시험으로 남으며 재실행 시 같은 후보/방향을 다시 시험합니다. 정차 평가나 유지 동작에서 명백히 실패한 후보는 정차·로그 저장 확인 후 거부로 기록하며 자동으로 재출발하지 않습니다. launch를 다시 실행하면 검증된 기존 최선에서 다음 후보를 제안합니다. **정지 평가나 보호 조건에서 기준 설정부터 실패하면 탐색을 종료**하고 원인을 먼저 해결해야 합니다. 20 cm 횡오차 평가 기준만 초과한 블록은 중단하지 않고 점수를 비교해 탐색을 계속합니다.

## 실행

Jetson/ROS 2 Humble 환경에서 기존 의존 패키지가 빌드되어 있어야 합니다. 기존 자동주행 launch와 동시에 실행하지 마세요. ROS 그래프의 기존 장치 소유 노드 검사와 이 패키지의 하드웨어 파일 잠금을 사용하지만, 다른 ROS_DOMAIN_ID나 임의 이름을 쓰는 외부 프로세스까지 장치 소유권을 보장하지는 않습니다.

```bash
# 저장소 루트, 기존 ROS/의존 workspace를 source한 터미널
colcon build --symlink-install --packages-select longitudinal_autotune
source install/setup.bash

ros2 launch longitudinal_autotune longitudinal_autotune.launch.py \
  direction:=cw \
  campaign_dir:=$HOME/.ros/longitudinal_autotune/track4m_001
```

포트 기본값은 `/dev/ttyTHS1`이며 `vesc_port:=...`로 변경합니다. 빌드 시 사용한 DepthAI 라이브러리 위치를 자동으로 읽으며 필요하면 `depthai_library_dir:=...`를 지정합니다. 지면 기준 측정·안정화 보정 동안 차량을 정지시킵니다.

시작은 항상 비활성입니다. 상태와 준비 여부를 확인한 뒤 시험을 시작합니다.

```bash
ros2 topic echo /longitudinal_autotune/status --full-length

# 다른 터미널: 실제 움직임 시작
ros2 service call /longitudinal_autotune/arm std_srvs/srv/SetBool '{data: true}'

# 시험 중단; 중단된 블록은 최적 후보로 채택하지 않음
ros2 service call /longitudinal_autotune/arm std_srvs/srv/SetBool '{data: false}'
```

모든 자동 명령은 `시험 게이트 → 기존 auto_handover → VESC bridge 하나`로 전달됩니다. **조이스틱 X는 기존과 같이 제동 후 수동 전환**합니다. 수동 전환·중단 후 자동 재활성화하지 않으며 launch를 다시 실행해야 합니다. BEV 속도 변경 UI는 시험 조건을 고정하기 위해 제어기에 연결하지 않습니다.

현재 방향 블록 완료 후 `needed: [ccw]`처럼 반대 방향이 표시되면 launch를 종료하고, 차량을 반대 방향으로 배치한 뒤 동일한 캠페인을 실행합니다.

```bash
ros2 launch longitudinal_autotune longitudinal_autotune.launch.py \
  direction:=ccw \
  campaign_dir:=$HOME/.ros/longitudinal_autotune/track4m_001
# 준비 후 /longitudinal_autotune/arm 호출
```

양방향 블록 완료 → 다음 후보를 현재 방향에서 시험 → 반대 방향이 필요할 때 정지 대기를 반복합니다. 도중에 완료하려면 먼저 중단하고 정차를 확인한 뒤 호출합니다.

```bash
ros2 service call /longitudinal_autotune/finish std_srvs/srv/Trigger '{}'
```

시험 YAML을 복사해 사용할 수 있습니다.

```bash
cp src/longitudinal_autotune/config/experiment.yaml /tmp/track4m_test.yaml
ros2 launch longitudinal_autotune longitudinal_autotune.launch.py \
  params_file:=/tmp/track4m_test.yaml direction:=cw \
  campaign_dir:=$HOME/.ros/longitudinal_autotune/track4m_002
```

기하·인식·속도·정책·모델·제어기 실행 파일이 달라지면 새 캠페인 디렉터리를 쓰세요. 캠페인 fingerprint가 설정/실행 파일/모델/실효 인식 설정의 변경을 검사합니다. 기존 생산용 YAML은 덮어쓰지 않습니다. 범퍼 거리 등 차량 고정값은 복사한 YAML의 `auto_control.ros__parameters`에 설정합니다.

UART ERPM만 읽는 기본 구성에는 배터리 전압·실제 모터 전류가 없습니다. 모르는 값을 0으로 만들지 않고 `null`로 기록합니다. 동일 배터리 상태·타이어·노면에서 비교하세요. 전압 확인을 필수로 하려면 기존 dynamics 모듈의 CAN 입력을 설정하고 `require_voltage: true`로 설정합니다. 전압이 있는 양방향 블록은 중앙값 차이가 0.6 V를 넘으면 승격하지 않습니다. 전압을 기록하지 못한 결과는 배터리 차이에 의한 오판 가능성이 남습니다.

## 기존에 잘 주행하던 설정을 기준으로 시험하기

평소 `auto_drive`에서 별도 YAML을 사용했다면 동일한 파일을 다음 인자로 지정합니다.

- `auto_control_params_file`: 기존 조향/경로/구동 제어 설정
- `camera_params_file`: 기존 카메라·IMU 설정
- `bev_params_file`: 기존 BEV 설정
- `line_detactor_params_file`: 기존 모델·중앙 경로 인식 설정

비워 두면 설치된 기본 파일을 사용합니다. 적용 순서는 설치 기본값 → 지정한 기존 YAML → `experiment.yaml`의 해당 노드 override → 시험 전용 토픽·기능·고정 속도 정책입니다. 따라서 선택한 파일의 조향 설정을 유지하면서 장애물·터널·신호등 추론은 사용하지 않고, 시험 속도와 제동 한도는 계속 적용합니다. 여섯 탐색 항목의 기준값도 선택한 제어 YAML에서 가져오며 탐색 범위를 벗어나면 시작을 거부합니다.

```bash
ros2 launch longitudinal_autotune longitudinal_autotune.launch.py \
  direction:=cw \
  auto_control_params_file:=/absolute/test_yaml/auto_control_test.yaml \
  camera_params_file:=/absolute/test_yaml/camera_config_test.yaml \
  bev_params_file:=/absolute/test_yaml/bev_config_test.yaml \
  line_detactor_params_file:=/absolute/test_yaml/line_detactor_test.yaml \
  campaign_dir:=$HOME/.ros/longitudinal_autotune/track4m_working_baseline_001
```

파일 경로는 Jetson에 실제로 존재하는 경로로 지정합니다. 파일 안의 `model_path`도 존재해야 합니다. 기존 캠페인과 실효 설정이 달라지면 새 `campaign_dir`를 사용합니다. `inputs.yaml`에 선택한 파일 경로와 실효 설정이 함께 기록됩니다.

### 첨부한 정상 주행 YAML과의 비교 결과

확인한 첨부본(`auto_control_test.yaml`, `camera_config_test.yaml`, `bev_config_test.yaml`, `line_detactor_test.yaml`) 기준입니다. 다른 경로의 오래된 로컬 사본과 구분합니다.

- 조향 설정은 설치 기본값과 같습니다: Stanley gain 1.3, softening 0.30 m/s, heading lookahead 0.05 m, servo inversion true. 경로 ROI·기하와 BEV 설정도 같습니다.
- `camera_config_test.yaml`은 `imu_stabilization_vehicle_motion_compensation_enabled: false`입니다. 기존 시험 기본값은 true였으므로, `experiment.yaml`에 false를 명시해 첨부본과 맞췄습니다. IMU 기울기 안정화 자체를 끄는 설정은 아닙니다.
- 첨부 `line_detactor_test.yaml`과의 차이는 결과 전용 프리뷰와 테마 설정이며, 시험 YAML이 이미 같은 값을 사용합니다.
- 시험의 최고 속도 0.8 m/s, 코너 횡가속도 기준 0.5 m/s², duty 상한 0.10, 정지 기준점 0.12 m는 종방향 시험을 위해 정한 고정 조건입니다. 첨부 일반 주행 설정의 1.8 m/s, 1.6 m/s², duty 0.12, 정지 여유 0.10 m와 의도적으로 다릅니다. 낮은 속도에서도 같은 조향기가 같은 추종 성능을 낸다고 보장하지는 않습니다.

카메라 설정 차이가 실제 이탈의 원인이라고 확정한 것은 아닙니다. 한 조건을 맞춘 뒤 새 기준 주행으로 확인합니다. `maximum_cte_m: 0.20`은 정책 v2부터 정지 조건이 아닌 평가 기준입니다. 별도의 `hard_cte_limit_m: 0.40`을 보호 중단에 사용합니다. 기존 실패 기록은 남기고, 변경한 카메라 설정의 시험은 새 캠페인 디렉터리를 사용합니다.

`traffic_light_test.yaml`, 장애물·LiDAR 설정은 이 시험에서 로드하지 않습니다. `centerline_preview_test.yaml`도 별도 프리뷰 설정이므로 `line_detactor_test.yaml` 위에 자동으로 덧씌우지 않습니다. 기존 실행에서 그 파일을 인식 설정으로 사용했다면 `line_detactor_params_file`에 실제 사용한 파일을 지정해야 합니다.

### 출발 직후 횡오차로 정지하는 경우

정책 v2는 20 cm 횡오차를 평가하며 계속 주행하고, 다음 실제 중단 원인을 따로 표시합니다. `speed_or_path_limit` 또는 20 cm의 `cross_track_error_limit`가 보이면 이전 버전이 실행되고 있는 것입니다. 정지 순간의 수치는 이후 제어기가 정지 값을 발행해도 유지합니다.

- `hard_cross_track_error_limit`: 횡오차 절댓값이 별도 `hard_cte_limit_m`(기본 0.40 m) 초과
- `overspeed_limit`: 속도가 고정 시험 최고 속도 + 0.3 m/s 초과
- `reverse_speed_limit`: 역방향 속도가 -0.03 m/s 미만

```bash
ros2 topic echo /longitudinal_autotune/fault
```

예: `hard_cross_track_error_limit value=-0.4119m limit=0.4000m`. 상태 토픽과 trace에도 같은 최초 원인·수치를 기록합니다. 0.212 m처럼 평가 기준 0.20 m는 넘지만 보호 한도 0.40 m 이내인 오차는 계속 주행하면서 측정합니다. 보호 한도는 차선 폭·차체 여유 공간을 추정한 값이나 안전 보증값이 아닌 시험 중단 설정이며, 자동 탐색으로 변경하지 않습니다. 이 패키지의 자동 탐색은 종방향 여섯 항목에 한정되며 조향 실패를 스스로 튜닝하지 않습니다.

## 횡오차를 관찰하며 평가하기: 정책 버전 2

```yaml
maximum_cte_m: 0.20       # 평가 기준: 초과해도 계속 주행하고 감점
hard_cte_limit_m: 0.40    # 별도 보호 중단 기준
score_weights:
  path_tracking: 2.0
```

순간적인 초과, 장시간 이탈, 복귀 여부를 비교하도록 최대/RMS 횡오차, 기준 초과 시간·비율, 초과 오차의 시간 적분, 최장 연속 초과 시간, 회복 횟수를 기록합니다. 경로 추종 점수는 RMS/기준값 + 최대값/기준값 + 초과 오차 시간 적분/기준값 + 초과 시간 비율이며 `path_tracking` 가중치를 곱합니다. 다른 항목이 같다면 더 크게·오래 벗어난 후보의 점수가 나빠집니다. 평가 기준 초과만으로 블록을 거부하지 않으므로 이후 후보와 비교할 수 있습니다.

```bash
ros2 topic echo /longitudinal_autotune/tracking --full-length
```

`--full-length`는 ROS 2 CLI가 긴 문자열을 128자에서 잘라 표시하는 것을 방지합니다.

이 토픽과 status의 `tracking`은 첫 기준 정차·유지 시간을 포함한 현재 블록 전체의 누적값입니다. 정차별 평가 결과는 HOLD를 제외한 해당 이동 구간의 추종 성적을 사용합니다. 경고 시작/복귀는 trace의 `tracking_warning`으로 기록합니다. 통신·인식 유실, 비정상 수치, 역주행, 과속, 정지선 초과, 별도 큰 이탈 한도 등의 중단 조건은 계속 유지합니다.

업데이트 후 새 터미널에서 환경을 source하고 실행 버전을 확인합니다.

```bash
source install/setup.bash
python3 -c 'import longitudinal_autotune.policy as p; print("policy_version=", p.POLICY_VERSION, "file=", p.__file__)'
```

출력은 `policy_version= 2`여야 합니다. 시작 로그에도 버전과 실제 세션 코드 경로가 표시됩니다. 버전 상수가 없거나 이전 정지 이름이 나오면 기존 launch를 완전히 종료하고 패키지를 다시 빌드·source한 뒤 실행합니다. 정책/목적함수가 변경됐으므로 새 캠페인(예: `track4m_004`)을 사용합니다. 이전 YAML을 복사한 경우 최신 설정의 `hard_cte_limit_m`, `score_weights.path_tracking`도 포함해야 합니다.

## 재실행 시 이전 시험 제어기가 남아 있는 경우

`Stop existing device/control nodes first: [('autotune_controller_1', '/longitudinal_autotune')]`는 중복 제어를 막기 위한 시작 검사입니다. 기존 시험 launch를 Ctrl+C로 종료한 뒤 다음처럼 시험 제어기만 확인·정리합니다.

```bash
pgrep -af 'auto_control_node.*__node:=autotune_controller_'
pkill -INT -f 'auto_control_node.*__node:=autotune_controller_[0-9]+'
pgrep -af 'auto_control_node.*__node:=autotune_controller_'
```

마지막 확인에 실제 프로세스가 나오면 남은 PID를 확인한 뒤 해당 프로세스만 종료합니다. 시작 검사를 우회하거나 모든 ROS 프로세스를 일괄 종료할 필요는 없습니다. PID가 없는데 이름만 보이면 DDS 그래프에서 종료가 반영될 때까지 잠시 기다린 뒤 재실행합니다.

수정된 세션은 설치된 `auto_control_node` 실행 파일을 직접 실행하고 소유 프로세스 그룹을 관리합니다. ROS context 종료로 disable/브레이크 토픽 발행에 실패해도 자식 프로세스·로그·잠금 정리를 finally에서 수행합니다. 이미 종료된 중간 프로세스 뒤에 남은 자식 그룹도 종료 대상으로 처리합니다. 강제 SIGKILL이나 장치/커널의 비정상 상태까지 정상 종료를 보장하는 기능은 아닙니다.

## 저장 결과

`campaign_dir` 아래:

- `campaign.json`: 후보·방향별 정차 평가·탐색 상태. 중단 후 재개에 사용.
- `candidate_parameters.yaml`: 현재 시험 후보. **최적값이 아닙니다.**
- `best_parameters.yaml`, `best_evaluation.json`: 양방향 반복 시험과 중단 조건을 통과한 기존 최선. 경로 추종의 평가 기준 충족 여부는 `tracking_targets_met`로 별도 표시하며, 미충족이면 상태를 `best_observed_with_tracking_deviations`로 저장합니다. 실패·중단 후보로 덮어쓰지 않습니다. 첫 양방향 기준 시험 통과 전에는 생성되지 않습니다.
- `runs/<실행 ID>/trace.jsonl`: 50 Hz 표본, 원본 차선 수신 시각/거리, 제어기 원시 모터 명령, 실제 게이트 통과 명령, 상태 전이, 정차 평가.
- `runs/<실행 ID>/inputs.yaml`, `controller_N.yaml/.log`, `controller_N/`: 실효 설정, 후보별 기존 제어기의 상세 CSV 기록. CSV에는 종방향 단계, 정지 거리 추적, 전류 제동 개입 등의 기존 진단값이 포함됩니다.

최적 YAML은 변경한 6개 파라미터만 포함합니다. 운영 설정에 자동 적용하지 않습니다. 고정 시험 설정과 합쳐 같은 조건으로 재검증한 후 운영 YAML에 반영해야 합니다. 정지 중 원자적으로 저장하며, 로그 worker의 flush/fsync가 완료되기 전에는 새 최적값을 승격하지 않습니다.

## 차량 응답 모델 추정

실차 기록이 쌓인 뒤 별도 오프라인 명령으로 실행합니다.

```bash
ros2 run longitudinal_autotune autotune_analyze \
  --trace /absolute/campaign/runs/run_cw/trace.jsonl \
  --trace /absolute/campaign/runs/run_ccw/trace.jsonl \
  --output /absolute/campaign/effective_vehicle_model.json
```

ERPM 속도를 고정 필터로 미분하고, **게이트를 통과한 실제 명령의 시간**을 사용해 다음 유효 가속도 모델과 지연을 추정합니다.

```text
a = b0 + b1*duty(t-delay) + b2*brake_current(t-delay)
       + b3*v + b4*v*abs(yaw_rate)
```

지연 후보는 0–240 ms, 20 ms 간격입니다. 속도 필터 지연도 포함된 유효 지연이므로 순수한 모터 지연으로 해석하지 않습니다. 충분한 구동·전류 제동·속도·회전 변화, 행렬 식별성, 계수 부호 검사를 통과해야 모델을 출력합니다. 최소 네 구간이 필요하고, 마지막 구간 묶음을 분리해 검증합니다. 학습 구간 내 그룹별 교차검증으로 지연을 선택하고 분리한 구간의 가속도 RMSE를 보고합니다. 검증은 같은 실험 조건의 구간 분리이며, 새 세션/노면의 일반화를 증명하지는 않습니다.

부족하면 `insufficient_identifiable_data`를 저장합니다. 데이터를 지어내거나 기존 기본값을 실측 모델이라고 저장하지 않습니다. 생성된 모델은 시뮬레이터 종방향 응답의 초기 후보로 사용할 수 있지만, 전체 횡동역학·타이어 슬립·절대 정지 위치를 재현하는 sim-to-real 모델은 후속 작업입니다. duty 감소만으로 회생 에너지와 전류 제동을 물리적으로 분리 식별하지도 않습니다.

## 검증

ROS 없이 평가·탐색·저장·게이트·프로세스 교체·조정기 콜백을 테스트할 수 있습니다.

```bash
python3 -m pytest src/longitudinal_autotune/test -q
```

ROS/CUDA/카메라/VESC를 사용한 빌드·실차 검증은 Jetson에서 별도로 필요합니다. 소프트웨어 시험 통과는 실차 최적값을 이미 얻었다는 뜻이 아닙니다.

## 시작 직후 status 토픽이 나오지 않는 경우

`ros2 topic echo /longitudinal_autotune/status`가 기다리기만 한다면 먼저 launch 터미널의 traceback을 확인합니다. 세션 노드가 시작 중 종료되면 status가 발행되지 않습니다.

- `imu_yaw_axis`가 `True / BOOL`이라고 나오면 문자열 타입이 전달 중 바뀐 것입니다. 수정 버전은 문자열을 명시적으로 따옴표 처리한 전용 ROS 파라미터 파일을 생성하고, launch에 파일 경로를 직접 전달합니다. 실효 숫자·BOOL 값은 그대로 유지합니다.
- `can't set attribute 'subscriptions'`는 `rclpy.Node`의 읽기 전용 속성과 이름이 충돌한 문제입니다. 세션의 별도 구독 목록 이름으로 수정했습니다.
- 여러 핵심 프로세스가 연이어 종료될 때 launch 종료 요청은 한 번만 보냅니다. 초기 오류 뒤의 SIGINT/종료 traceback은 최초 시작 오류와 구분해서 확인합니다.

수정본을 받은 뒤 이 패키지만 다시 빌드하고 source합니다. 이 두 시작 오류 때문에 캠페인 기록을 삭제할 필요는 없습니다.

```bash
git pull --ff-only origin 1030ML
colcon build --symlink-install --packages-select longitudinal_autotune
source install/setup.bash
```

Jetson의 sourced ROS 환경에서는 실제 rcl YAML 파서의 문자열 타입 보존을 장치 없이 확인할 수 있습니다.

```bash
python3 -m pytest src/longitudinal_autotune/test/test_ros_parameter_runtime.py -q
```
