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
| 코너·주행 | 곡률 속도 상한 초과량, 목표 속도 추종 오차, 횡오차, 구간 시간 기록 |
| 데이터 | 차선/ERPM/IMU/제어 명령 수명, 중복·역순 차선 timestamp, 로그 누락 검사 |

정지 유지 중에는 **시험 대상 제어기의 명령을 그대로 전달**하여 꿈틀거림을 숨기지 않습니다. 데이터 이상·한도 위반·움직임을 발견하면 시험 게이트가 제동으로 전환합니다. 정상 주행은 기존의 duty 감소 → 감속 반응 확인 → 부족할 때 전류 제동 절차를 그대로 사용합니다. 보호 중단의 제동은 점수 대상 정상 감속과 구분합니다.

정지 거리와 유지 이동량은 BEV 거리 및 ERPM 적분에 의한 **추정값**입니다. 카메라 기하, 앞차축→범퍼 거리, 타이어/구동비가 틀리거나 바퀴가 미끄러지면 실제 위치 오차와 다릅니다. 이 설정들은 탐색 중 바꾸지 않습니다. 실측 없이 절대 정지 정확도를 인증하거나 질량·마찰계수를 개별 식별했다고 할 수 없습니다.

바꾸는 파라미터는 다음 6개뿐입니다.

1. `longitudinal_planning_deceleration_mps2`
2. `duty_rise_rate_per_sec`
3. `duty_fall_rate_per_sec`
4. `brake_current_rise_amps_per_sec`
5. `brake_current_fall_amps_per_sec`
6. `corner_planning_acceleration_mps2`

기준 설정을 먼저 양방향에서 검증한 뒤, 현재 최선 설정의 한 항목을 제한 범위 내에서 ± 변경하는 좌표 탐색을 합니다. 개선이 없으면 변경 폭을 줄입니다. 최대 20개 후보입니다. 목적함수는 정지 위치·jerk·유지 이동·구동/제동 전환·추종·주행 시간·코너 과속의 가중합이며, 양방향 8개 점수의 평균 + 0.3×표준편차로 비교합니다. 기본 2% 이상 개선된 후보만 승격합니다. 통계적 유의성이나 수렴을 보장하는 기준은 아니며 반복 횟수를 늘려 재검증할 수 있습니다.

후보 교체는 방향 블록이 끝나고 정차가 확인된 상태에서만 수행합니다. 출력 게이트가 정지 상태를 유지하면서 기존 `auto_control` 프로세스를 종료하고 새 YAML로 재시작합니다. 이전 프로세스가 살아 있는 동안 다음 프로세스를 시작하지 않습니다. 다음 방향의 기록이 필요한 경우 정지한 채 기다립니다. 양방향 검증이 끝나면 현재 방향의 다음 후보 블록은 자동으로 진행할 수 있습니다.

센서·차선·통신·로그·운전자 중단은 불완전 시험으로 남으며 재실행 시 같은 후보/방향을 다시 시험합니다. 정차 평가나 유지 동작에서 명백히 실패한 후보는 정차·로그 저장 확인 후 거부로 기록하며 자동으로 재출발하지 않습니다. launch를 다시 실행하면 검증된 기존 최선에서 다음 후보를 제안합니다. **기준 설정부터 실패하면 탐색을 종료**하고 원인을 먼저 해결해야 합니다.

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
ros2 topic echo /longitudinal_autotune/status

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

## 저장 결과

`campaign_dir` 아래:

- `campaign.json`: 후보·방향별 정차 평가·탐색 상태. 중단 후 재개에 사용.
- `candidate_parameters.yaml`: 현재 시험 후보. **최적값이 아닙니다.**
- `best_parameters.yaml`, `best_evaluation.json`: 양방향 반복 검증을 통과한 기존 최선. 실패·중단 후보로 덮어쓰지 않습니다. 첫 양방향 기준 시험 통과 전에는 생성되지 않습니다.
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
