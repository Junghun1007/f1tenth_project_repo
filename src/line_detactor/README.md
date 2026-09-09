# line_detactor

`bev_processor`가 BEV 변환 직후 발행하는 `/camera/image_bev`를 구독해
Fast-SCNN HighRes로 왼쪽·오른쪽 차선을 추론하는 독립 프리뷰 패키지다.
실행 경로는 신호등 패키지와 같은 `ONNX → TensorRT FP32`이며 PyTorch를
설치하지 않는다. 기존 차선 검출·제어 토픽을 구독하거나 변경하지 않고 결과
토픽도 발행하지 않는다.

패키지 이름 `line_detactor`는 요청된 이름을 그대로 유지한다.

## 모델과 GPU 경로

- 입력 토픽: `/camera/image_bev`, `bgr8`, 가로 120 × 세로 300
- ONNX 입력: RGB `[0,1]`, FP32 BCHW `[1,3,300,120]`
- ONNX 출력: left/right logits `[1,2,300,120]`
- 채널 0 left: 파란색, 채널 1 right: 빨간색
- 각 채널에 독립 sigmoid와 기본 threshold `0.5` 적용
- 두 채널이 겹치면 로짓이 큰 한쪽 색만 사용하며 자홍색은 만들지 않음
- softmax, 곡선 피팅, 연결, 작은 조각 제거는 적용하지 않음

패키지에 `best.pt`에서 고정 120×300 입력으로 내보낸
`fast_scnn_highres_120x300_batch_1.onnx`가 포함된다. 내보내기 검증에서 PT와
ONNX의 threshold 0.5 마스크 결과는 일치했다.

프레임별 경로는 다음과 같다.

```text
ROS BGR8 → pinned host → H2D → CUDA BGR→RGB/CHW 및 /255
         → TensorRT FP32 → CUDA threshold/overlay → BGR8 D2H → preview
```

TensorRT의 큰 logits 배열은 CPU로 복사하지 않는다. 최종 프리뷰 BGR8 한 장만
D2H하며, 최신 프레임 하나만 보관하고 최대 30 FPS로 처리한다. 하단에는
동기화된 순수 TensorRT 실행 시간, 그 역수인 model FPS, 실제 preview FPS가
표시된다. 로그는 `H2D+preprocess`, `pure-inference`,
`postprocess+D2H` 평균/최댓값을 각각 출력한다.

## Jetson 의존성과 빌드

JetPack에 포함된 CUDA·TensorRT 개발 패키지와 OpenCV가 필요하다. `torch`나
Python CUDA wheel은 필요하지 않다. TensorRT 헤더가 없다면 다음 패키지를
설치한다.

```bash
sudo apt update
sudo apt install libnvinfer-dev libnvonnxparsers-dev libnvinfer-plugin-dev
```

빌드 명령은 다음과 같다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build --packages-select line_detactor
source install/setup.bash
```

## BEV와 모델 프리뷰 실행

터미널 1에서 기존 룰베이스 차선 인식과 BEV 자체 프리뷰를 끄고 원본 BEV만
발행한다. 사용하는 YAML에서 `publish_enabled: true`여야 한다.

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

첫 실행은 ONNX에서 장치 전용 FP32 엔진을 만들기 때문에 수 분이 걸릴 수 있다.
기본 캐시는 ONNX 옆의
`*.trt<TensorRT-major>.fp32.engine`에 저장되며 다음 실행부터 재사용된다.
다른 Jetson 또는 TensorRT 버전에서 만든 엔진은 복사하지 말고 해당 장치에서
다시 생성한다.

## 수동주행과 함께 실행

`auto_drive.launch.py`는 실행하지 않는다. 별도 터미널에서 수동주행을 켠다.

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
않는다. `Q`, `ESC` 또는 프리뷰 창 닫기로 종료한다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `input_topic` | `/camera/image_bev` | 원본 BGR8 BEV 토픽 |
| `model_path` | 패키지 내 ONNX | 고정 입력 Fast-SCNN HighRes ONNX |
| `engine_cache_path` | 빈 문자열 | 비우면 ONNX 옆에 장치 전용 엔진 저장 |
| `tensorrt_workspace_size_mb` | `1024` | 최초 FP32 엔진 생성 workspace |
| `model_input_width/height` | `120/300` | ONNX와 BEV의 고정 크기 |
| `mask_threshold` | `0.5` | left/right 독립 sigmoid 임계값 |
| `overlay_alpha` | `0.75` | 차선 색상 오버레이 불투명도 |
| `warmup_iterations` | `10` | 시작 시 TensorRT 워밍업 횟수 |
| `preview_fps` | `30.0` | 최대 추론·프리뷰 FPS |
| `preview_scale` | `2.0` | 프리뷰 창 확대 배율 |
| `status_log_interval_sec` | `1.0` | 성능 로그 간격 |
