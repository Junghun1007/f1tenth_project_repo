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
- 원본 두 채널이 겹치면 로짓이 큰 쪽에 픽셀을 배정하며 자홍색은 만들지 않음
- 선택적으로 프레임별 자연 3차 스무딩 스플라인 보정 적용 (기본 OFF)
- softmax, 검출 공백 연결, 작은 조각 삭제, 시간축 필터는 적용하지 않음

패키지에 `best.pt`에서 고정 120×300 입력으로 내보낸
`fast_scnn_highres_120x300_batch_1.onnx`가 포함된다. 내보내기 검증에서 PT와
ONNX의 threshold 0.5 마스크 결과는 일치했다.

프레임별 경로는 다음과 같다.

```text
ROS BGR8 → pinned host → H2D → CUDA BGR→RGB/CHW 및 /255
         → TensorRT FP32 → CUDA threshold/overlay → BGR8 D2H → preview
```

TensorRT의 큰 logits 배열은 CPU로 복사하지 않는다. 보정 OFF에서는 최종
프리뷰 BGR8 한 장만 D2H한다. ON에서는 추론과 오버레이 사이에 다음 단계가
추가된다. 120×300 기준 행 정보는 편도 7,200바이트이며 logits는 GPU에 남는다.

```text
CUDA 행별 중심/신뢰도 추출 → 행 정보 D2H → CPU 스플라인/이동량 제한
                         → 보정 행 정보 H2D → CUDA 보정 오버레이
```

최신 프레임 하나만 보관하고 최대 30 FPS로 처리한다. 하단에는 `infer` 순수
TensorRT 실행 시간, 그 역수인 `model FPS`, `correct` 보정 시간 또는 `OFF`,
보정량 제한 설정, 실제 `view FPS`가 표시된다.

`correct`는 추론 완료 후부터 GPU 행 추출, 행 정보 D2H, CPU 피팅/제한,
보정 정보 H2D 완료까지의 wall-clock 시간이다. 보정 마스크의 이동/합성과
프리뷰 D2H는 별도 `postprocess+D2H`에 포함된다. 보정 OFF의 측정값은 0이다.
로그는 `H2D+preprocess`, `pure-inference`, `correction`, `postprocess+D2H`
평균/최댓값을 각각 출력한다.

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

## YAML 지정 및 보정 ON/OFF

저장소 루트에서 아래처럼 파라미터 파일을 지정한다. YAML의 노드 키는
`line_detactor: ros__parameters:` 구조를 유지한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  smoothing_enabled:=true
```

원본 마스크 프리뷰로 실행:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  smoothing_enabled:=false
```

보정 강도/최대 이동량 지정:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml" \
  smoothing_enabled:=true \
  smoothing_strength:=8.0 \
  smoothing_correction_limit_enabled:=true \
  smoothing_max_correction_px:=2.0
```

`smoothing_correction_limit_enabled:=false`로 이동량 제한만 끌 수도 있다.
우선순위는 **명시한 launch 인자 > 지정 YAML > 노드 기본값**이다. 비어 있는
launch 인자는 YAML을 덮어쓰지 않는다. `params_file`을 생략하면 설치된 기본
YAML을 사용한다. 파라미터는 시작 시 읽으므로 변경 후 재실행한다.

## 보정 방식과 범위

1. GPU에서 좌우 마스크를 분리하고 각 행의 확률 가중 중심을 추출한다.
   하나의 연속 픽셀 덩어리가 있고 좌우 화면 경계에 닿지 않는 행만 사용한다.
2. 공백, 여러 덩어리, 경계 접촉, 인접 행 중심의 과도한 점프에서 구간을 나눈다.
   짧거나 애매한 구간의 마스크는 원본으로 유지한다.
3. 좌우/구간별로 `검출 중심과의 가중 오차 + strength × 곡률 에너지`를
   최소화하는 자연 3차 스무딩 스플라인을 계산한다. 신뢰도는 행 안의 평균
   sigmoid 확률이며 수치 안정성을 위해 가중치 하한 0.05를 적용한다.
   행 수에 비례하는 대역 Cholesky 풀이를 사용하며 외부 피팅 라이브러리는 없다.
4. 제한 ON에서는 구간 내 최대 수평 이동량이 `smoothing_max_correction_px`를
   넘지 않도록 구간 전체 보정량을 같은 비율로 줄인다. 모든 거리 단위는
   확대 전 BEV 픽셀이다. 강도 또는 최대 이동량이 0이면 중심 이동이 없다
   (최대 이동량은 제한 ON일 때만 적용).
5. 매 행의 원본 차선 마스크를 보정량만큼 수평 이동한다. 소수 픽셀 이동은
   선형 보간으로 가장자리 불투명도에 반영해 작은 보정도 표시한다. 따라서
   제한은 중심의 연속 좌표 이동량에 적용되며 가장자리 픽셀에는 부분 색상이
   생길 수 있다. 이동 후 좌우가 겹치면 보간된 확률 기여도가 큰 한쪽을 표시한다.

프레임 간 상태나 공백 보간은 없고, 별도 폴리곤/제어 경로 토픽도 생성하지
않는다. 원본 차선 폭을 유지하는 중심 보정이므로 폭 자체의 찌그러짐을 따로
고치지는 않는다. 화면 밖으로 이동한 부분은 잘린다. 넓은 구간의 잘못된 검출을
실제 커브와 구분하는 기능은 없으며 기본 수치는 실차 튜닝 전 시작값이다.

이번 보정 기능의 빌드/실행/테스트는 수행하지 않았다.

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
| `smoothing_enabled` | `false` | 프레임별 스플라인 보정 ON/OFF |
| `smoothing_strength` | `8.0` | 평활화 강도, 0이면 원본 중심 유지 |
| `smoothing_correction_limit_enabled` | `true` | 최대 보정량 제한 ON/OFF |
| `smoothing_max_correction_px` | `2.0` | 최대 수평 이동량, BEV 픽셀 |
| `smoothing_max_row_jump_px` | `4.0` | 인접 행 중심 차이가 초과하면 구간 분리 |
| `smoothing_min_segment_rows` | `12` | 보정에 필요한 최소 연속 행 수 |
| `warmup_iterations` | `10` | 시작 시 TensorRT 워밍업 횟수 |
| `preview_fps` | `30.0` | 최대 추론·프리뷰 FPS |
| `preview_scale` | `2.0` | 프리뷰 창 확대 배율 |
| `status_log_interval_sec` | `1.0` | 성능 로그 간격 |
