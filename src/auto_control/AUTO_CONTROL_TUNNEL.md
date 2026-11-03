# auto_control 터널 주행 통합

터널 밖 RGB 차선 주행 → TCN ENTRY → 터널 IR 차선 주행 → TCN EXIT → RGB 복귀를
`auto_control`이 관리한다. 상태·경로 선택기는 이 패키지의 `tunnel_lane_selector`이며,
별도 ROS 패키지나 전용 launch 없이 기존 `vehicle_bringup auto_drive.launch.py`에서
`tunnel_enabled:=true`로 활성화한다. 하나의 카메라 세션·자동제어기·VESC 경로를 사용한다. 항상 터널 밖에서 시작한다.
기존 `tunnel_tcn_test`, `ir_bev_line_detection`의 독립 launch는 그대로 사용할 수 있다.
통합 launch와 기존 카메라/자동/수동 launch를 동시에 실행하지 않는다.

## 처리 빈도와 카메라 설정

| 상태 | RGB 차선 | IR BEV·차선 | 제어 경로 |
|---|---:|---:|---|
| OUTSIDE | 최대 80 FPS | 대기 0 FPS | RGB |
| INSIDE | 최대 5 FPS | 최대 30 FPS | IR |

수치는 처리 상한/요청값이며 Jetson 연산 부하에 따라 실측 FPS가 낮아질 수 있다.
중복 프레임으로 FPS를 채우지 않는다. 두 TensorRT 모델은 시작 시 로딩·워밍업하며
TCN 이벤트로 프로세스·카메라를 재시작하지 않는다.

RGB CAM_A는 80 FPS로 유지하며, 원본 NV12를 촬영 시각 기준 약 30 FPS로 따로 전달해
TCN의 기존 밝기 특징과 10 Hz 추론·2.5초 시작 워밍업을 유지한다. TCN은 터널 안에서도
실행해야 출구를 감지할 수 있다. 통합 TCN 프리뷰는 끈다.

IR CAM_B는 양쪽 모드에서 1280×800 / 30 FPS로 촬영한다. 외부에서는 밝기가 다른
장애물용 영상에 차선 추론을 하지 않는다. 카메라 연결·보정과 모델 로딩·워밍업은 유지하고,
노출과 무관한 투영 LUT만 미리 준비한다. 외부 IR BEV 변환·모델 입력은 0 FPS이며,
TCN ENTRY 수신 즉시 INSIDE로 바꾸고 30 FPS로 활성화한다. RGB의 시작 지면 측정·IMU
기준 자세를 IR 투영에도 공유하며 실제 CAM_B 출력의 내부·외부 파라미터를 사용한다.
IR BEV 컴포넌트가 밝기를 유지한 BGR 3채널도 직접 발행하며 기존 IR 모델·차선 후처리를 거친다.
통합 실행은 독립 테스트의 Python 어댑터를 시작하지 않는다.

ENTRY 요청 시 IR 노출 30000μs / ISO800 / dot OFF / flood0.5를 적용한다.
설정 명령 후 최소 150ms를 제외하고 실제 노출·ISO 메타데이터가 일치하는 프레임만
터널 IR 입력으로 인정한다. EXIT 수신 즉시 RGB를 선택하고 IR 자동 노출과 기존 외부
장애물 dot/flood 설정을 복원한다. 외부 IR 노출은 외부 장애물 설정을 따르므로 해당
조건의 차선 결과는 생성하지 않는다.
선택 소스는 즉시 바뀌지만 IR의 첫 결과까지는 최소 150ms 노출 설정 구간과
영상·BEV·추론 시간이 필요하다. 이 구간에 RGB 결과로 전환을 지연하지 않는다.

외부 장애물 기능을 켠 경우 같은 CAM_B/C를 공유한다. 통합 모드의 스테레오 센서와
depth 요청도 **30 FPS**로 설정해 터널의 30ms 노출을 지원한다. 외부 RGB 80 FPS와는
별개이며, 기존 외부 depth의 60 FPS 요청을 유지하는 구성은 아니다.
터널에서는 host depth 전달과 제어기의 장애물 회피·감속 및 신호등 정지를 중지한다.
OAK 내부에 구성된 StereoDepth 노드 자체는 유지한다. 출구에서 설정 복원 후 새 depth만
외부 처리에 전달하며, 이전 장애물/정지선 제어 상태를 지운다.

## 전환과 유실 처리

TCN ENTRY를 수신하면 즉시 `OUTSIDE/RGB → INSIDE/IR`, EXIT를 수신하면 즉시
`INSIDE/IR → OUTSIDE/RGB`로 바꾼다. ENTERING/EXITING 대기 상태와 새 경로 3회 확인,
전환 후보 경로의 기하 검사·요청 이후 촬영 확인 조건을 제거했다.
선택된 소스의 첫 결과부터 원본 그대로 제어기에 전달한다. 저장된 결과를 재발행하거나
원본 timestamp를 현재 시각으로 바꾸지 않는다. 중복·역순·오래된 결과는 전달하지 않으며
기존 C++ 제어기의 경로 timeout/capture-age/ERPM 및 경로 검증은 그대로 적용된다.

TCN이 WARMUP 중이거나 0.5초 넘게 결과가 없으면 제어 경로 전달을 보류한다.
따라서 첫 출발은 TCN과 차선 경로가 모두 준비된 후 가능하다.
TCN의 이벤트 시각과 실제 터널 경계가 일치하는지는 차량에서 검증해야 한다.
이벤트 결과 한 개가 유실되어도 이후 TCN 결과에 유지되는 inside/outside 상태로 요청을 복구한다.
반대 이벤트가 발생하면 해당 소스로 즉시 다시 전환한다.

X 버튼의 AUTO → STOPPING → MANUAL 전환은 기존 방식 그대로다.
터널 이벤트는 자동주행 활성화나 수동 전환 해제를 요청하지 않는다.

## 단일 프리뷰와 터널 HUD

RGB 검출기만 GUI 창을 소유하며, IR 검출기는 별도 창을 열지 않는다. 선택된 경로가
IR이면 같은 창에 최신 IR 결과를 표시한다. 오래된 IR 이미지는 대기 화면으로 바꾼다.
IR 결과는 외부 장애물 플래너의 RGB 차선 입력을 덮어쓰지 않는다.

기존 `line_detactor_test.yaml`의 `line_detactor.ros__parameters`에서 다음을 설정한다.

```yaml
preview_enabled: true
bev_theme_enable: false
preview_result_only_enabled: true
```

`preview_result_only_enabled: true`는 검은 배경에 차선·정지선·중앙 경로를 표시한다.
`false`는 실제 BEV 영상 위에 결과를 겹친다. 선택은 RGB와 IR에 동일하게 적용한다.
기존 실행 명령에 `preview_result_only_enabled:=false`가 있으면 YAML보다 우선하므로
제거하거나 `true`로 바꾼다. 이 설정은 재실행해서 적용한다.
초기 지면 보정 ROI 창은 별개이며 `bev_config_test.yaml`의
`measurement_roi_preview_enabled: false`로 끌 수 있다.

신호등 상태 옆에 `Tunnel: OUTSIDE / INSIDE`을 표시한다.
바로 아래에는 TCN `avg … FPS`, LIVE/WARMUP/STALE 상태와
ENTRY/EXIT 점수를 표시한다. 평균 FPS는 LIVE 보고의 TensorRT+전송 wall time을 평균한 뒤
`1000 / 평균 ms`로 계산한다. 주행 루프나 실제 실행 주기를 의미하지 않는다.
워밍업·입력 유실 때는 FPS를 `--`로 표시한다.
일반 RGB/IR 프리뷰의 `Duty`는 `/auto/current_duty`의 제어 명령값을 소수점 4자리로 표시한다
(예: `0.0750` = 7.5%). 0.5초 이상 수신되지 않거나 유효하지 않으면 `--`로 표시한다.
`line_detactor.preview_duty_topic`으로 구독 토픽을 변경할 수 있으며, 기존 YAML에는 추가하지 않아도 된다.
`monitor_only`에서는 계산된 명령값이며 실제 모터 출력 피드백을 뜻하지 않는다.

## 시작 시 모델 검사

`auto_drive.launch.py`는 실제 활성화된 RGB 차선 모델과, 터널 활성화 시 IR 차선·TCN,
신호등 활성화 시 신호등 모델을 `model_startup_check`로 확인한다. 장애물은 신경망 모델을 쓰지 않는다.
각 모델은 엔진 로드/생성 후 시험 추론을 성공해야 `READY`를 발행한다.
차선 모델은 `warmup_iterations: 0`이어도 최소 1회 실행하며,
신호등은 실제 운영의 NV12→CUDA→TensorRT 입력 경로를 시험한다.
TCN은 고정 입력 크기와 유한한 출력값까지 확인한다.

터미널에는 `[MODEL CHECK] PASS <노드명>`과 모델 경로(차선은 엔진 경로도)가 표시되고,
모든 활성 모델이 준비되면 `[MODEL CHECK] ALL READY (N models)`를 표시한다.
실패 보고는 `FAIL`, 보고가 없는 모델은 10초마다 `Waiting`으로 표시한다.
기본 300초 안에 준비되지 않으면 `NOT READY` 오류를 표시하면서 계속 감시한다.
콜드 TensorRT 엔진 빌드는 수 분 걸릴 수 있다. 이 검사는 진단용이며 자동주행 허가를
추가로 차단하지 않는다. 모델 준비는 카메라 로드·영상 입력·실제 검출 정확도 검증과 별개다.

## 차량에서 빌드·실행

ROS 2 Humble / CUDA·TensorRT / DepthAI C++ 3.6+가 준비된 Jetson의 워크스페이스
루트(`src`의 부모)에서 실행한다. 새 공용 라이브러리가 있으므로 의존 패키지도 빌드한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 먼저 연산·인식·전환만 확인. 기존 외부 주행 YAML 인자를 그대로 추가할 수 있다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true \
  auto_control_mode:=monitor_only preview_enabled:=true speed_mps:=1.0

# 위 실행 종료 후 주행 준비. 기본적으로 출발은 보류한다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true \
  auto_enabled:=false preview_enabled:=true speed_mps:=1.0
```

`speed_mps`는 0보다 큰 유한한 목표 속도(m/s)이며 0.3 상한은 없다.
생략하면 기존 외부 제어 YAML/launch의 최소·최대 속도를 상속한다.
IR 전용 launch와 마찬가지로 실제 속도는 기존 ERPM 피드백으로 제어한다.
`auto_enabled`는 기존 auto_drive 및 선택한 제어 YAML 값을 따른다. 위 주행 준비 명령은
명시적으로 false를 지정한다. 보정을 끝내고 TCN LIVE·차선을 확인한 뒤
다른 source된 터미널에서 활성화한다.

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool 'data: true'
ros2 topic echo /tunnel/state
ros2 topic echo /tunnel/selected_source
ros2 topic hz /tunnel/lane_result
```

기존 인자 `camera_params_file`, `bev_params_file`, `line_detactor_params_file`,
`auto_control_params_file`, `obstacle_params_file`, `vesc_port`, CAN 인자를 함께 쓸 수 있다.
IR 학습과 일치하도록 통합 BEV는 앞차축 기준 120×300, 전방 0~3m, 좌우 ±0.6m,
1cm/pixel을 요구한다. 기본 회피 제어가 활성화된 설정에서 외부 장애물도 제외하려면
`obstacles_enabled:=false avoidance_control_enabled:=false`를 함께 지정한다.
기존 RGB 전용 동작을 유지하도록 `tunnel_enabled` 기본값은 false다.
공용 IR 모델과 검증 보고서는 `line_detactor/models`에 있으며, 단독 IR 테스트와 함께 사용한다.

## 로컬 검증

```bash
PYTHONPATH=src/auto_control python3 -m unittest discover \
  -s src/auto_control/test/tunnel -v
c++ -std=c++17 -Isrc/line_detactor/include \
  src/line_detactor/test/frame_rate_gate_test.cpp -o /tmp/frame_rate_gate_test
/tmp/frame_rate_gate_test
```

이 검사는 즉시 상태 전환, 오래된/역순 결과 거부, 모델 준비 상태 집계와 처리 빈도 제한을 확인한다.
이 개발 환경에서는 ROS 빌드·DepthAI 동시 출력·Jetson TensorRT·실차 전환은 검증하지 않았다.
기존 단독 터널 주행 성공은 이 통합 경로의 검증을 대신하지 않는다.
