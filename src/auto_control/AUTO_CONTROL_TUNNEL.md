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
| OUTSIDE | 최대 80 FPS | 최대 5 FPS | RGB |
| ENTERING | 최대 80 FPS | 최대 30 FPS | 새 IR 경로 확인 전까지 RGB |
| INSIDE | 최대 5 FPS | 최대 30 FPS | IR |
| EXITING | 최대 80 FPS | 최대 30 FPS | 새 RGB 경로 확인 전까지 IR |

수치는 처리 상한/요청값이며 Jetson 연산 부하에 따라 실측 FPS가 낮아질 수 있다.
중복 프레임으로 FPS를 채우지 않는다. 두 TensorRT 모델은 시작 시 로딩·워밍업하며
TCN 이벤트로 프로세스·카메라를 재시작하지 않는다.

RGB CAM_A는 80 FPS로 유지하며, 원본 NV12를 촬영 시각 기준 약 30 FPS로 따로 전달해
TCN의 기존 밝기 특징과 10 Hz 추론·2.5초 시작 워밍업을 유지한다. TCN은 터널 안에서도
실행해야 출구를 감지할 수 있다. 통합 TCN 프리뷰는 끈다.

IR CAM_B는 양쪽 모드에서 1280×800 / 30 FPS로 촬영한다. 저빈도 대기는 센서 FPS를
바꾸는 대신 IR BEV 변환과 모델 입력 빈도를 제한한다. RGB의 시작 지면 측정·IMU
기준 자세를 IR 투영에도 공유하며 실제 CAM_B 출력의 내부·외부 파라미터를 사용한다.
IR BEV 컴포넌트가 밝기를 유지한 BGR 3채널도 직접 발행하며 기존 IR 모델·차선 후처리를 거친다.
통합 실행은 독립 테스트의 Python 어댑터를 시작하지 않는다.

ENTRY 요청 시 IR 노출 30000μs / ISO800 / dot OFF / flood0.5를 적용한다.
설정 명령 후 최소 150ms를 제외하고 실제 노출·ISO 메타데이터가 일치하는 프레임만
터널 IR 입력으로 인정한다. EXIT 후에는 RGB 경로 선택이 끝날 때까지 IR 설정을 유지하고,
그 뒤 IR 자동 노출과 기존 외부 장애물 dot/flood 설정을 복원한다.
따라서 **외부 IR 대기 영상의 노출은 외부 장애물 설정을 따르며**, 그 결과를 입구에서
그대로 제어 입력으로 선택하지 않는다.

외부 장애물 기능을 켠 경우 같은 CAM_B/C를 공유한다. 통합 모드의 스테레오 센서와
depth 요청도 **30 FPS**로 설정해 터널의 30ms 노출을 지원한다. 외부 RGB 80 FPS와는
별개이며, 기존 외부 depth의 60 FPS 요청을 유지하는 구성은 아니다.
터널에서는 host depth 전달과 제어기의 장애물 회피·감속 및 신호등 정지를 중지한다.
OAK 내부에 구성된 StereoDepth 노드 자체는 유지한다. 출구에서 설정 복원 후 새 depth만
외부 처리에 전달하며, 이전 장애물/정지선 제어 상태를 지운다.

## 전환과 유실 처리

TCN 이벤트는 전환 요청이다. 요청을 수신한 뒤 촬영된 새로운 경로가 연속 3회
기하·최신성 검사를 통과해야 실제 입력을 바꾼다. 검사 기준은 선택한 외부 제어기의
경로 점 수·길이·ROI·인접 간격 기준을 사용한다. 기존 C++ 제어기가 최종 검증한다.
전환 준비 동안 기존 소스의 새로운 결과만 전달하며, 저장된 결과를 재발행하거나
원본 timestamp를 현재 시각으로 바꾸지 않는다. 기존 경로까지 유실되면 기존 제어기의
경로 timeout/capture-age/ERPM 정지 조건을 그대로 적용한다.

TCN이 WARMUP 중이거나 0.5초 넘게 결과가 없으면 제어 경로 전달을 보류한다.
따라서 첫 출발은 TCN과 차선 경로가 모두 준비된 후 가능하다.
TCN의 이벤트 시각과 실제 터널 경계가 일치하는지는 차량에서 검증해야 한다.
이벤트 결과 한 개가 유실되어도 이후 TCN 결과에 유지되는 inside/outside 상태로 요청을 복구한다.
전환 준비 중 반대 이벤트가 발생하면 최신 요청을 따르므로 미완료 진입에 갇히지 않는다.

X 버튼의 AUTO → STOPPING → MANUAL 전환은 기존 방식 그대로다.
터널 이벤트는 자동주행 활성화나 수동 전환 해제를 요청하지 않는다.

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

이 검사는 상태 전환, 오래된/역순 결과 거부, 경로 기하와 처리 빈도 제한을 확인한다.
이 개발 환경에서는 ROS 빌드·DepthAI 동시 출력·Jetson TensorRT·실차 전환은 검증하지 않았다.
기존 단독 터널 주행 성공은 이 통합 경로의 검증을 대신하지 않는다.
