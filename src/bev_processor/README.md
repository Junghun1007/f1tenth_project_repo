# bev_processor

`camera_driver`가 보내는 왜곡 보정된 1280x720 NV12 프레임을 Jetson CUDA로
직접 BEV 변환하는 ROS 2 C++ 패키지다. 결과는 200x482 BGR8이며
`/camera/image_bev`로 발행하거나 OpenCV 창에 표시할 수 있다.

## 처리 구조

- OAK에서 Jetson까지는 BGR888i 대신 NV12로 전송한다.
- 카메라와 BEV를 동일한 `component_container_mt`에 적재하고
  intra-process 통신을 사용한다.
- 시작할 때 지면 역투영 LUT를 한 번 계산해 GPU 메모리에 올린다.
- CUDA 커널 하나가 NV12 Y/UV 샘플링, 색 변환, BEV 워핑을 결합해 수행한다.
- 전체 1280x720 BGR 프레임은 만들지 않고 200x482 BEV만 CPU로 내려받는다.
- OpenCV 프리뷰 복사본에 0.5 m 좌표 격자를 표시하며 ROS BEV 출력은 변경하지 않는다.
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

카메라와 GPU BEV를 같은 프로세스에서 실행한다.

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

카메라 장착 위치와 각도는 launch 옵션으로 실행할 때마다 조정할 수 있다.
이 값으로 시작 시 BEV LUT를 생성하므로 값을 바꾼 뒤 launch를 다시 실행한다.

```bash
ros2 launch bev_processor camera_bev.launch.py \
  camera_x_m:=0.0 \
  camera_y_m:=0.0 \
  camera_z_m:=0.20 \
  camera_roll_deg:=0.0 \
  camera_downward_pitch_deg:=14.0 \
  camera_yaw_deg:=0.0
```

시작 로그에는 `transport=NV12`와 사용 중인 CUDA GPU 이름이 표시된다.
상태 로그의 의미는 다음과 같다.

- 카메라 `capture`: DepthAI에서 Jetson으로 실제 수신한 FPS
- BEV `input`: BEV 컴포넌트가 받은 NV12 FPS
- BEV `processed`: CUDA BEV 변환이 완료된 FPS
- BEV `skipped`: 처리 중 최신 입력으로 교체된 오래된 프레임 수
- BEV `gpu`: NV12 업로드, CUDA 커널, 작은 BEV 다운로드를 합친 시간

## 독립 프로세스 실행

문제 분리용으로 카메라와 BEV를 별도 프로세스로 실행할 수도 있다. 이 경우
전체 NV12 프레임이 DDS를 통과하므로 통합 launch보다 복사 비용이 크다.

```bash
# 터미널 1
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=false \
  publish_enabled:=true

# 터미널 2
ros2 launch bev_processor bev_processor.launch.py
```

## 현재 보정값

- 입력/K_rect: 1280x720
- `fx=561.400939941`, `fy=561.136352539`
- `cx=643.032653809`, `cy=352.621124268`
- 카메라 높이 0.20 m, 하향각 14도, yaw 0도
- 전방 0.18~5.0 m, 좌우 -1.0~1.0 m
- 0.01 m/px, 출력 200x482

카메라 위치나 각도가 바뀌면 `config/bev_config.yaml`의 외부 파라미터를
다시 측정해야 한다.
