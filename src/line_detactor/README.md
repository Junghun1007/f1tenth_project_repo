# line_detactor

`bev_processor`가 BEV 변환 직후 발행하는 `/camera/image_bev`를 구독해
Fast-SCNN HighRes 체크포인트로 왼쪽·오른쪽 차선을 독립적으로 추론하는
ROS 2 프리뷰 패키지다. 기존 차선 검출 및 제어 토픽을 구독하거나 변경하지
않으며, 결과 토픽도 발행하지 않는다.

패키지 이름 `line_detactor`는 요청된 이름을 그대로 사용한다.

## 입출력

- 입력: `/camera/image_bev`, `bgr8`, 가로 120 × 세로 300
- 모델 입력: RGB `[0,1]`, FP32 BCHW `[1,3,300,120]`
- 모델 출력: `[1,2,300,120]` logits
- 채널 0: left, 파란색
- 채널 1: right, 빨간색
- 각 채널에 독립 sigmoid와 기본 threshold `0.5` 적용
- softmax, argmax, 곡선 피팅, 연결, 작은 조각 제거를 적용하지 않음

두 채널이 같은 픽셀에서 모두 임계값을 넘으면 프리뷰에서만 로짓이 더 높은
채널의 파랑 또는 빨강을 사용하며 별도의 중첩 색상은 표시하지 않는다.

CUDA 실행에서는 pinned BGR8 입력만 H2D한 뒤 BGR→RGB, CHW, `/255`,
sigmoid, threshold와 색상 오버레이를 GPU에서 처리한다. 프리뷰에 필요한
최종 BGR8 한 장만 D2H한다. 프리뷰는 최신 프레임만 처리하며 최대 30 FPS로
제한되어 오래된 프레임이 쌓이지 않는다.

하단에는 다음 값을 표시한다.

- `inference`: CUDA 동기화된 순수 `model(tensor)['out']` forward 시간
- `model FPS`: 순수 forward 시간의 역수
- `preview FPS`: 실제 추론·표시 FPS

터미널 로그는 `preprocess+H2D`, `pure-inference`, `postprocess+D2H`의
평균/최댓값을 별도로 표시한다.

## Jetson 의존성

JetPack과 일치하는 NVIDIA CUDA PyTorch가 필요하다. 일반 PyPI `torch`로
덮어쓰지 않는다. 다음 결과에서 CUDA가 `True`여야 한다.

```bash
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

ROS의 `python3-numpy`, `python3-opencv`도 필요하다.

## 빌드

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build --packages-select line_detactor
source install/setup.bash
```

## BEV와 모델 프리뷰 실행

터미널 1에서 기존 룰베이스 차선 검출을 끄고 원본 BEV만 발행한다.
`publish_enabled: true`인 BEV YAML을 사용해야 한다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=false \
  lane_seed_detection_enabled:=false
```

초기 depth·IMU 측정이 끝날 때까지 차량을 정지시킨다. 터미널 2에서 모델
프리뷰를 실행한다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch line_detactor line_detactor.launch.py
```

## 수동주행과 함께 실행

`auto_drive.launch.py`는 실행하지 않는다. 별도 터미널에서 기존 수동주행을
실행한다. CANable dynamics가 필요한 현재 구성의 예시는 다음과 같다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  vesc_port:=/dev/ttyTHS1 \
  controller_name_contains:=8BitDo \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

`line_detactor`는 프리뷰만 수행하며 VESC, 조향, 자동주행 토픽을 발행하지
않는다. `Q`, `ESC` 또는 프리뷰 창 닫기로 모델 프리뷰 노드를 종료한다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `input_topic` | `/camera/image_bev` | 원본 BGR8 BEV 토픽 |
| `model_path` | 패키지 내 `models/best.pt` | Fast-SCNN HighRes 체크포인트 |
| `device` | `cuda:0` | PyTorch 실행 장치; CUDA 불가 시 오류 |
| `amp_enabled` | `true` | CUDA 자동 혼합 정밀도 |
| `mask_threshold` | `0.5` | left/right 독립 sigmoid 임계값 |
| `warmup_iterations` | `20` | 시작 시 모델 워밍업 횟수 |
| `preview_fps` | `30.0` | 최대 추론·프리뷰 FPS |
| `preview_scale` | `2.0` | 120×358 프리뷰 창 확대 배율 |
| `status_log_interval_sec` | `1.0` | 성능 로그 간격 |
