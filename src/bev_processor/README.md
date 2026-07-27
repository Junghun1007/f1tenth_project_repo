# bev_processor

`camera_driver`가 보내는 왜곡 보정된 1280x720 NV12 프레임을 Jetson CUDA로
직접 BEV 변환하는 ROS 2 C++ 패키지다. 결과는 200x282 BGR8이며
`/camera/image_bev`로 발행하거나 OpenCV 창에 표시할 수 있다.

## 처리 구조

- OAK에서 Jetson까지는 BGR888i 대신 NV12로 전송한다.
- 카메라와 BEV를 동일한 `component_container_mt`에 적재하고
  intra-process 통신을 사용한다.
- 통합 launch는 BEV 컴포넌트를 먼저 로드한다. BEV가 OAK 스테레오와 내부
  IMU로 카메라 높이, roll, 하향 pitch를 측정하고 OAK를 해제한 다음
  `camera_driver`가 RGB 스트리밍을 시작한다.
- 측정된 외부 파라미터로 지면 역투영 LUT를 한 번 계산해 GPU 메모리에
  올리고 해당 실행 동안 고정한다.
- 시작 측정을 끄면 `camera_z_m`, `camera_roll_deg`,
  `camera_downward_pitch_deg` config 값을 그대로 사용한다.
- CUDA 커널 하나가 NV12 Y/UV 샘플링, 색 변환, BEV 워핑을 결합해 수행한다.
- 전체 1280x720 BGR 프레임은 만들지 않고 200x282 BEV만 CPU로 내려받는다.
- OpenCV 프리뷰 바깥 여백에 좌표 글자를 표시하고 영상에는 0.1 m
  연회색 격자만 그리며 ROS BEV 출력은 변경하지 않는다.
- 입력이 처리보다 빠르면 큐를 쌓지 않고 최신 프레임으로 교체해 지연을
  제한한다.

카메라의 원본 프리뷰는 NV12를 CPU BGR로 바꾸므로 통합 실행 시 자동으로
꺼진다. 화면에는 이미 크기가 작은 BEV 결과만 표시한다.

## 요구 사항과 빌드

ROS 2 Humble, DepthAI C++ 3.x, OpenCV 4와 JetPack CUDA Toolkit이 필요하다.

```bash
nvcc --version
nvidia-smi 2>/dev/null || true
```

워크스페이스 루트에서 Release 모드로 빌드한다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

CUDA 컴파일러를 자동으로 찾지 못하면 JetPack 설치 경로를 지정한다.

```bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
```

## 권장 실행

차량을 정지시키고 영상 중앙이 평평한 노면을 보도록 한 뒤 카메라와 GPU
BEV를 같은 프로세스에서 실행한다. 시작 시 짧은 측정 파이프라인이 먼저
실행되고, 측정 완료 로그를 출력한 뒤 RGB 카메라와 BEV 처리가 시작된다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bev_processor camera_bev.launch.py
```

BEV 창과 BEV 토픽 발행을 각각 끌 수 있다.

```bash
ros2 launch bev_processor camera_bev.launch.py \
  preview_enabled:=false \
  publish_enabled:=true
```

측정을 사용하지 않고 config/launch 외부 파라미터를 직접 적용하려면
`startup_measurement_enabled:=false`로 실행한다. X/Y 위치와 yaw는 시작
측정으로 구할 수 없으므로 측정 모드에서도 항상 config 값을 사용한다.

```bash
ros2 launch bev_processor camera_bev.launch.py \
  startup_measurement_enabled:=false
```

이때 높이, roll, 하향 pitch를 포함한 카메라 외부 파라미터는 launch의
기본값으로 덮어쓰지 않고 `config/bev_config.yaml` 값을 그대로 사용한다.

시작 로그에는 `transport=NV12`와 사용 중인 CUDA GPU 이름이 표시된다.
측정 모드에서는 BEV 시작 전에 다음 형태의 로그가 먼저 표시된다.

```text
BEV_STARTUP_MEASUREMENT: height=0.2031m, roll=1.200deg,
pitch=-14.100deg, downward_pitch=14.100deg
```

상태 로그의 의미는 다음과 같다.

- 카메라 `capture`: DepthAI에서 Jetson으로 실제 수신한 FPS
- BEV `input`: BEV 컴포넌트가 받은 NV12 FPS
- BEV `processed`: CUDA BEV 변환이 완료된 FPS
- BEV `skipped`: 처리 중 최신 입력으로 교체된 오래된 프레임 수
- BEV `gpu`: NV12 업로드, CUDA 커널, 작은 BEV 다운로드를 합친 시간
- BEV `extrinsics`: `measured` 또는 `config`와 실제 적용 중인
  높이/roll/하향 pitch

## 독립 프로세스 실행

문제 분리용으로 카메라와 BEV를 별도 프로세스로 실행할 수도 있다. 이 경우
전체 NV12 프레임이 DDS를 통과하므로 통합 launch보다 복사 비용이 크다.
시작 측정을 사용하려면 OAK를 점유한 카메라 프로세스가 없는 상태에서
BEV를 먼저 실행해 측정을 끝낸 뒤 카메라를 실행해야 한다. 일반 사용에서는
이 순서를 자동으로 보장하는 결합 launch를 사용한다.

```bash
# 터미널 1
ros2 launch bev_processor bev_processor.launch.py

# BEV_STARTUP_MEASUREMENT 로그가 나온 뒤 터미널 2
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=false \
  publish_enabled:=true
```

## 현재 보정값

- 입력/K_rect: 1280x720
- `fx=561.400939941`, `fy=561.136352539`
- `cx=643.032653809`, `cy=352.621124268`
- config 모드 카메라 높이 0.20 m, roll 0도, 하향각 14도, 고정 yaw 0도
- 전방 0.18~3.0 m, 좌우 -1.0~1.0 m
- 0.01 m/px, 출력 200x282

카메라 X/Y 위치와 차량 기준 yaw가 바뀌면 `config/bev_config.yaml`을 다시
조정해야 한다. 측정 모드에서는 높이와 roll/하향 pitch를 시작할 때
OAK 스테레오+IMU로 결정하고 주행 중에는 고정한다.
