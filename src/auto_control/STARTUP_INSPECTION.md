# 출발 전 점검과 시작 승인

`vehicle_bringup auto_drive.launch.py`는 일반 실행에서 출발 전 점검 창을 자동으로 연다.
기존 YAML이나 `auto_enabled:=true`를 사용해도 점검과 사용자 승인 전에는 자율주행 duty를 허용하지 않는다.

## 표시 순서

1. CAMERA ATTITUDE: 처음 카메라가 비추는 측정 영상을 표시한다. 자동 지면 모드는
   깊이 영상과 빨간 측정 ROI를 보여 주고, 수동 높이 모드는 RGB 영상과 IMU 자세 측정을
   사용한다. 실제 측정 완료 후 높이/roll/pitch와 PASS를 표시한다.
2. INFERENCE MODELS: 현재 실행에서 필요한 RGB/IR/TCN/신호등 모델별 WAITING,
   LOADING, READY, FAILED를 표시한다. 엔진 생성·시험 추론을 완료해야 READY다.
3. DEPARTURE: 카메라 측정, 모델 준비 heartbeat, VESC 연결, 신선한 ERPM,
   유효한 최신 차선과 기존 제어 입력 조건이 모두 충족되면 START 버튼을 활성화한다.
   클릭 또는 Enter 입력 시 C++ 제어기가 그 시점의 조건을 다시 검사한다.
4. C++ 제어기의 승인이 확인되면 점검 창만 닫는다. 주행 중 BEV 프리뷰는 유지한다.

측정이 끝난 카메라 영상은 마지막 영상으로 고정하고 frozen으로 표시한다.
점검 창은 별도 프로세스/메인 스레드에서 그리며 기존 BEV·모델 로딩을 막지 않는다.
UI 영상 출력은 관측용이며 측정 품질 기준이나 카메라 EEPROM은 바꾸지 않는다.

## duty 허용 조건

GUI는 duty 토픽이나 /auto/enabled를 발행하지 않는다. /auto_control/start 서비스에
시작 요청만 보내고, 자동 제어 노드가 자체 점검을 통과한 경우에만 승인한다.
최종 duty 발행 함수에도 승인 여부 검사를 적용한다. GUI를 닫거나 오류가 나도
승인이 생기지 않는다. 브레이크/긴급 정지와 기존 차선/장애물/신호등 조건은 유지한다.

카메라 재측정, 모델 준비 신호 중단/실패, VESC 연결 끊김, ERPM timeout,
자동 제어 disable 또는 수동 전환은 승인을 취소한다. 복구되어도 자동으로
다시 출발하지 않고 새 START 요청이 필요하다.
일시적인 차선 입력 누락/신호등 정지 등 기존 주행 조건은 기존 로직을 따른다.
auto_enabled:=false는 여전히 disable이며 START 버튼이 이를 덮어쓰지 않는다.

## 수동과 성능 측정

manual_test:=true는 기존처럼 자동 제어기를 monitor_only로 실행하고, 수동 컨트롤러의
duty 경로를 변경하지 않는다. X 버튼으로 수동 전환할 때는 기존 STOPPING 제동·정지 확인을
거쳐 MANUAL로 진입한다. 수동 상태에서는 시작 버튼 단계를 통과 처리하며 카메라/모델
표시도 통과하면 점검 창을 닫는다. 별도의 manual_drive.launch.py는 점검 창을 시작하지 않는다.
성능 측정 모드는 기존처럼 자동 출력이 억제되며 점검 GUI를 열지 않는다.

## 반영과 빌드

OAK 공용 측정 헤더가 변경되므로 전체 ROS 워크스페이스를 재빌드한다.
DepthAI SDK 재설치는 필요 없다. 엔진 캐시가 있는 install/을 삭제하지 않는다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo &&
git pull --ff-only origin 1030ML &&
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH &&
source /opt/ros/humble/setup.bash &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release &&
source install/setup.bash
```

GUI에는 ROS Python 환경의 cv2/numpy가 필요하다. 패키지 의존성은 python3-opencv,
sensor_msgs, std_srvs로 선언했다. 디스플레이가 없으면 GUI가 시작 승인을 내리지 않는다.
수동 서비스 요청도 C++의 같은 점검을 거친다.

진단 인터페이스:

```bash
ros2 topic echo /bev_processor/startup_status --qos-durability transient_local --once
ros2 topic echo /auto_control/startup_state --qos-durability transient_local --once
ros2 topic echo /perception/models_ready --qos-durability transient_local --once
```

## 검증 범위

ROS 없이 실제 C++ 정지 조건과 최종 duty guard를 컴파일해 승인 전 차단,
너무 이른 시작 거부, 연결/모델/속도 정보 소실 시 재승인 요구와 수동 모드를 검증한다.
Python 테스트는 UI 활성화/승인 응답 대기, 상태 freshness와 창 렌더링을 확인한다.
실제 OAK 영상, ROS 서비스·QoS 연결과 차량 작동은 Jetson에서 전체 빌드 후 확인한다.
