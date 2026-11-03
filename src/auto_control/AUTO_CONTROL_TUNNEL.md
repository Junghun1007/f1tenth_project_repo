# auto_control 터널 주행 통합

터널 밖 RGB 차선 주행 → TCN ENTRY → 터널 IR 차선 주행 → TCN EXIT → RGB 복귀를
`auto_control`이 관리한다. 상태·경로 선택기는 이 패키지의 `tunnel_lane_selector`이며,
별도 ROS 패키지나 전용 launch 없이 기존 `vehicle_bringup auto_drive.launch.py`에서
`tunnel_enabled:=true`로 활성화한다. 하나의 카메라 세션·자동제어기·VESC 경로를 사용한다. 항상 터널 밖에서 시작한다.
기존 `tunnel_tcn_test`, `ir_bev_line_detection`의 독립 launch는 그대로 사용할 수 있다.
통합 launch와 기존 카메라/자동/수동 launch를 동시에 실행하지 않는다.

## 현재 기본: 독립 IR 테스트와 맞춘 파이프라인 검증

`tunnel_validation_mode:=true`가 기본이다. 기존 실행 명령·외부 YAML을 그대로 써도
적용되며, TCN ENTRY/EXIT가 선택 소스를 즉시 바꾼다. 파이프라인 검증 후 기존 차단
조건을 다시 비교하려면 `tunnel_validation_mode:=false`로 실행한다.

| 항목 | 기본 검증 모드 `true` | 기존 조건 `false` |
|---|---|---|
| IR 노출 변경 후 150ms 대기 | 없음 | 적용 |
| IR 노출 30000±500μs / ISO800 메타데이터 일치 | 관측 로그만 출력 | 불일치 프레임 제외 |
| IR 촬영 후 250ms 제한 / 터널 프로필 준비 확인 | 없음 | 적용 |
| IR BEV timestamp | 독립 테스트 기본값과 같은 BEV 발행 시각 | 실제 촬영 시각 |
| TCN 결과 시각 검사 | 새 LIVE 상태 보고로 즉시 전환 | 촬영 후 0.2~0.25초 범위 검사 |
| TCN WARMUP·보고 유실 중 차선 전달 | 마지막 선택 소스 계속 전달 | 전달 보류 |
| 카메라 간 timestamp 비교 | 소스별 중복·역순만 검사 | 두 소스 사이에도 역순 검사 |

촬영 시각은 원본 DirectIrFrame과 `[IR CAMERA]`/`[IR BEV]` 진단에서 유지한다.
BEV 발행 시각을 쓰면 제어기의 capture-age 값은 촬영→BEV 변환까지의 지연을 포함하지
않는다. 결과를 계속 재발행하는 것은 아니며, 새 카메라 프레임당 한 번만 생성한다.
모델·노출 설정값·BEV 규격·흑백 3채널 복제·추론 후처리는 독립 IR 테스트와 같다.

독립 테스트에도 있는 IMU 보정 유효성 검사와 C++ 제어기의 중앙선 기하/수신 timeout,
ERPM/VESC 조건, X 버튼 수동 전환은 유지한다. 모든 활성 모델의 시험 추론 완료를
기다리는 조건도 유지한다. RGB 외부 주행·신호등 모델과 속도 설정은 바꾸지 않는다.
통합은 RGB 지면/IMU 기준과 카메라 세션을 공유하므로 독립 실행과 같은 연산 부하를
보장하는 구성은 아니다. IMU 제외가 계속되면 `rejected(.../imu)` 로그로 확인한다.

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
기본 검증 모드는 설정 명령 전후의 새 프레임도 즉시 투영하며, 노출 메타데이터가
일치할 때까지 기다리지 않는다. 따라서 전환 직후 짧게 외부 노출의 이미지가 포함될 수 있다.
`false` 모드에서는 150ms 대기와 실제 노출·ISO 일치 검사를 적용한다.
EXIT 수신 즉시 RGB를 선택하고 IR 자동 노출과 기존 외부 장애물 dot/flood 설정을 복원한다.
소스 선택은 즉시지만 카메라가 노출을 바꾸고 첫 IR 영상을 추론하는 물리적 시간은 필요하다.

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
선택된 소스의 첫 결과부터 전달하며 저장된 결과를 재발행하지 않는다.
기본 검증 모드는 TCN이 워밍업 중이거나 보고가 끊겨도 마지막 선택 소스의 차선 결과를
계속 전달한다. `false` 모드는 TCN이 WARMUP 중이거나 0.5초 넘게 보고가 없으면 보류한다.
IR 결과는 기본 검증 모드에서 독립 IR 드라이버처럼 BEV 발행 시각을 쓰며,
`false` 모드에서는 실제 촬영 시각을 사용하고 최신성·카메라 간 역순을 검사한다.
기존 C++ 제어기의 경로 timeout/기하/ERPM 조건은 양쪽 모드에 적용된다.
TCN의 이벤트 시각과 실제 터널 경계가 일치하는지는 차량에서 검증해야 한다.
이벤트 결과 한 개가 유실되어도 이후 TCN 결과에 유지되는 inside/outside 상태로 요청을 복구한다.
반대 이벤트가 발생하면 해당 소스로 즉시 다시 전환한다.

X 버튼의 AUTO → STOPPING → MANUAL 전환은 기존 방식 그대로다.
터널 이벤트는 자동주행 활성화나 수동 전환 해제를 요청하지 않는다.

## 단일 프리뷰와 터널 HUD

RGB 검출기만 GUI 창을 소유하며, IR 검출기는 별도 창을 열지 않는다. 선택된 경로가
IR이면 같은 창에 IR 결과를 표시한다. 최초 결과 전에는 대기 화면을 표시하고,
수신 후 결과 timestamp 기준 0.25초를 넘기면 마지막 화면을 유지하며 `IR STALE: …ms`를 표시한다.
이 화면 유지는 GUI에만 적용하며, 선택기·제어기에 이전 경로를 재전달하지 않는다.
다음 터널 진입 시에는 이전 터널의 화면을 비운다.
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
콜드 TensorRT 엔진 빌드는 수 분 걸릴 수 있다. `auto_drive`는 모든 활성 모델이 준비될 때까지
자동제어기를 `waiting_for_models`로 유지해 구동 duty를 출력하지 않는다. 300초가 지나도
자동으로 해제하지 않는다. 준비 상태는 `/perception/models_ready`로 1초마다 발행하고,
로드 실패·재로딩 보고 또는 3초 넘는 상태 수신 중단 시 다시 주행을 차단한다.
단독 제어기/기존 IR 독립 테스트는 이 감시 노드를 실행하지 않으므로 기본적으로 이 조건을
요구하지 않는다. 통합 launch가 `model_startup_check_required: true`를 직접 설정하며,
기존 테스트 YAML에 파라미터를 추가할 필요는 없다.
모델 준비는 카메라 로드·영상 입력·실제 검출 정확도 검증과 별개다.

## IR 독립 테스트와 통합 경로의 차이 및 끊김 확인

같은 IR FP32 ONNX 모델, LEFT 1280×800/30 FPS, 30ms/ISO800/dot0/flood0.5,
120×300 BEV, 동일한 흑백→BGR 복제를 사용한다. 통합에서는 RGB·TCN·신호등·depth와
하나의 카메라 세션/연산 자원을 공유하고, RGB에서 측정한 지면·IMU 기준 자세를 재사용한다.
독립 IR은 카메라 설정을 시작부터 적용하지만 통합은 진입 시 변경한다.
독립 IR 드라이버 기본 `use_sensor_timestamps: false`는 BEV 발행 시각을 쓴다.
통합 기본 검증 모드도 같은 방식이며, `tunnel_validation_mode:=false`만 촬영 시각을 쓴다. 독립 프리뷰에는 0.25초 초과 시 검게
지우는 조건이 없어서 같은 처리 지연이 다르게 보일 수 있다.

통합 IR 입력은 카메라의 새로운 촬영 프레임마다 처리하고, 센서가 이미 30 FPS이므로
호스트 도착 시각으로 다시 30 FPS를 샘플링하지 않는다. IR 검출기 내부의 추가 FPS 제한도
끄고 BEV 생성 측에서만 입력 빈도를 관리한다. 모델 자체와 IMU 검증은 같다. 노출 메타데이터 일치 검사는 검증 모드에서 적용하지 않는다.

터널 안에서 2초마다 다음 로그로 끊김 위치를 확인한다.

- `[IR CAMERA] received/published`: 실제 카메라 수신과 투영 입력 전달 빈도.
  `rejected(time/exposure/imu)`와 실제 노출/ISO로 제외 원인을 확인한다.
- `[IR BEV] output/source_age/status`: 모델 입력 생성 빈도, 촬영 나이, 입력·보정 대기 상태.
- `[ir_line_detactor] FPS: input/processed`: IR 추론 입력과 처리 빈도.
- 화면 `IR STALE`: 마지막 표시 결과 timestamp가 오래됐다는 뜻이며 새 검출 결과를 뜻하지 않는다.

이 로그 없이 실제 입력 유실이 노출·IMU·연산 지연 중 어디서 발생했는지 단정할 수 없다.

## DepthAI 실행 라이브러리 경로

공용 BEV/카메라 컨테이너는 빌드 때 선택된 `depthai::core` 라이브러리 디렉터리를
`bev_processor/depthai_runtime_dir.txt`에 기록하고, 실행 시 컨테이너의
`LD_LIBRARY_PATH` 맨 앞에 자동 적용한다. 다른 프로세스의 환경은 바꾸지 않는다.
필요하면 `auto_drive.launch.py depthai_library_dir:=/absolute/sdk/lib`로 덮어쓸 수 있다.
파일이 없거나 디렉터리가 사라졌다면 launch가 원인을 표시한다.
`undefined symbol: _ZTIN3dai10DeviceNodeE`가 계속되면 DepthAI를 사용하는 패키지들을
동일한 `-Ddepthai_DIR=.../lib/cmake/depthai`로 캐시를 지우고 다시 빌드해야 한다.
모델의 `ALL READY`는 카메라 컴포넌트 로드 성공을 의미하지 않는다.

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
