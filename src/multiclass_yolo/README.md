# multiclass_yolo

OAK CAM_A 영상을 받아 YOLOX 4개 클래스의 검출 결과를 보는 독립 ROS 2 프리뷰 패키지입니다.
기존 주행/신호등 패키지는 수정하지 않습니다. 이 노드는 OAK 장치를 직접 열므로 같은 장치를 사용하는
`camera_driver`, `bev_processor`, `traffic_detection_test` 등은 종료한 후 실행하세요.

## 모델 계약

기본 모델은 `multiclass_yolox_tiny_512x320_holdout10.onnx`입니다. 하단 80% 전용 `roi80_448x224.onnx`도 선택할 수 있습니다.
동봉된 `.metadata.json`과 `.labels.txt`에 입력·출력 형식과 클래스 순서가 기록되어 있습니다.

- 입력: 기본 모델 FP32 `[1,3,320,512]`, 448×224 모델 FP32 `[1,3,224,448]`. **BGR**, 0–255, 정규화 없음
- 리사이즈: 종횡비 유지, 좌상단 배치, 오른쪽/아래를 114로 채움
- 출력: 기본 모델 FP32 `[1,3360,9]`, 448×224 모델 FP32 `[1,2058,9]`
- 행: `[cx,cy,w,h,objectness,con,truck,sedan,traffic_light]`
- 좌표는 모델 입력 픽셀 기준으로 이미 디코딩되어 있음. sigmoid/grid decode를 다시 적용하지 않음
- 가장 높은 클래스 확률을 선택하고 `objectness × class_probability`에 클래스별 임계값 적용
- 클래스별 NMS (기본 IoU 0.65), 원본 영상 좌표로 복원

사용자가 처음 설명한 RGB 학습 여부보다 전달된 export 메타데이터의 BGR 계약을 따릅니다.
학습/export 코드가 이 메타데이터와 다르다면 채널 순서를 다시 확인해야 합니다.
`con`은 제공된 라벨 철자를 그대로 유지합니다.

## 빌드

기존 워크스페이스 루트(`src`, `cmake`가 있는 폴더)에서 Jetson 데스크톱 터미널로 실행합니다.
기존 프로젝트와 같은 ROS 2 Humble 및 `/usr/local` DepthAI 3.6 공유 SDK를 사용합니다.
TensorRT 개발 라이브러리(`libnvinfer-dev`, `libnvonnxparsers-dev`, `libnvinfer-plugin-dev`),
CUDA toolkit, OpenCV core/dnn/highgui/imgproc가 필요합니다. 기존 `traffic_detection_test`를 빌드할 수 있는 환경을 기준으로 합니다.
SDK를 새로 빌드하거나 교체하지 않습니다. 설치 경로가 다르면 기존 `DEPTHAI_SETUP.md`를 참고하세요.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select multiclass_yolo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 실행 및 입력 경로 비교

```bash
# NV12 → CUDA fused 전처리 → BGR 텐서 → TensorRT FP16
ros2 launch multiclass_yolo preview.launch.py input_format:=nv12

# 같은 NV12 카메라 영상 → CPU RGB 영상 → BGR 텐서 → TensorRT FP16
ros2 launch multiclass_yolo preview.launch.py input_format:=rgb

# 초기 클래스별 임계값, NMS 조절
ros2 launch multiclass_yolo preview.launch.py \
  input_format:=nv12 con_threshold:=0.30 truck_threshold:=0.40 \
  sedan_threshold:=0.35 traffic_light_threshold:=0.20 nms_threshold:=0.65

# CPU 진단 경로 (30 FPS 성능용 기본 설정은 아님)
ros2 launch multiclass_yolo preview.launch.py \
  input_format:=rgb inference_backend:=CPU engine_precision:=fp32
```

`input_format`은 **전처리 시작 형식**을 고릅니다. 카메라 USB 전송 형식은 두 모드 모두 NV12로 고정하여
같은 전송 조건에서 비교합니다. `rgb`에서는 OpenCV NV12→RGB 변환을 하고, 모델 blob을 만들 때
BGR 채널 순서로 바꿉니다. `nv12`에서는 NV12를 GPU로 복사한 뒤 색상 변환·리사이즈·패딩·NCHW 작성을
CUDA 커널에서 처리하여 추론용 전체 RGB 중간 영상을 만들지 않습니다.
**NV12 자체를 학습된 ONNX에 직접 넣는 방식은 아닙니다.**

프리뷰 표시는 두 모드 모두 CPU BGR 변환이 필요합니다. 따라서 NV12 모드도 CPU 변환 비용이 0이 아니며,
측정된 출력 FPS는 프리뷰 비용까지 포함합니다. zero-copy 파이프라인은 아니고 NV12 H2D 복사는 남습니다.
CUDA 색 변환은 기존 프로젝트와 동일한 **BT.601 limited range**를 가정하고 plane pitch를 반영합니다.
실제 카메라 스트림이 다른 색 행렬/range를 쓴다면 이 계수를 맞춘 뒤 검출 정확도를 비교해야 합니다.

첫 실행은 ONNX로 TensorRT 엔진을 빌드하므로 시간이 걸립니다. 엔진은 기본적으로
`~/.cache/multiclass_yolo/` 아래 모델 경로/크기/수정시각과 정밀도로 구분하여 저장합니다.
다른 ONNX는 `model_path:=/absolute/model.onnx`, 캐시 위치는 `engine_cache_path:=/absolute/model.engine`으로
지정할 수 있습니다. 다른 GPU/JetPack으로 옮길 때 기존 엔진 대신 ONNX에서 다시 생성하세요.

## 프리뷰

- 10% 확대된 영상과 클래스명·점수·박스
- 왼쪽 검출 목록에 현재 프레임의 클래스, 검출 점수, 적용된 클래스 임계값 표시. 목록이 길면 `Prev` / `Next`로 이동
- `AVG input`: 첫 호스트 수신부터 현재까지의 평균 수신 FPS `(수신 수 - 1) / 경과시간`
- `AVG detection`: 같은 시점부터 검출을 마친 프레임 수 / 경과시간
- 처음 0.5초는 FPS를 0으로 표시하여 초기값 급등을 피함. 엔진 빌드/카메라 시작 대기는 제외
- `total`: 현재 프레임의 처리 시작부터 영상 준비까지 걸린 시간 (GUI 제출 제외)
- `input/display color`: 추론용 CPU 색 변환 시간과 표시용 CPU 색 변환 시간
- `skipped`: 호스트 수신 후 최신 프레임 우선 처리 때문에 검출하지 않은 프레임 수 (장치/USB 드롭 수와 다름)
- `Detection thresholds`: `con`, `truck`, `sedan`, `traffic_light`별 검출 임계값을 0–1로 입력
- `Bottom ROI %`: 화면 하단부터 검출할 높이를 정수 퍼센트로 입력 (1–100, 기본 65). 노란 선이 검출 영역의 상단을 표시
- `Camera FPS`: 카메라 프레임 속도를 숫자로 입력. F 키로 이 입력 칸에 바로 이동
- 입력 칸을 클릭한 뒤 숫자를 입력하고 Enter 또는 `Apply`로 적용
- Q / ESC / 창 닫기로 종료. 숫자 입력 중에는 ESC로 입력을 취소

GUI에서 입력한 값은 현재 실행에만 적용됩니다. 재실행 때 유지하려면 실행 인수 또는 YAML을 수정하세요.
YAML로 실행하려면 아래처럼 `ros2 run`에 전달합니다 (기본 launch는 launch 인수를 사용합니다).

```bash
ros2 run multiclass_yolo multiclass_yolo_node --ros-args \
  --params-file src/multiclass_yolo/config/preview.yaml
```

카메라 기본값은 CAM_A, 1280×800, 30 FPS, 기존 프로젝트처럼 undistort 활성화입니다.
1280×800 프레임에서 ROI 65%는 하단 520행입니다. ROI와 클래스별 임계값은 다음 추론부터 적용됩니다.
시작 ROI는 launch 인수 `roi_height_percent:=65` 또는 YAML의 `roi_height_percent`로 지정합니다.
100%에서는 전체 프레임을 검출합니다.

프리뷰에는 전체 영상이 나오고 검출 박스는 원본 좌표에 표시됩니다. NV12/CUDA 경로는 ROI에 해당하는
픽셀을 색차 샘플 경계에 맞춰 GPU로 전송합니다. 카메라 USB 전송과 프리뷰용 색 변환에는 전체 프레임을 사용하며,
선택한 TensorRT 모델의 입력 크기와 추론 연산량은 고정됩니다. 로그의 `upload`에서 GPU 입력 전송 시간을 비교할 수 있습니다.

FPS 입력 범위는 연결된 카메라의 1280×800 센서 모드를 따릅니다. 적용할 때 카메라 스트림을
다시 여는 동안 영상이 잠시 멈춥니다. TensorRT 엔진은 유지되고, 새 프레임부터 평균 FPS를 다시 계산합니다.
변경에 실패하면 기존 FPS로 스트림 재개를 시도합니다. 다음 실행의 초기값은 `sensor_fps`로 지정합니다.
`width`, `height`, `sensor_fps`, `undistort_enabled`를 launch 인수로 바꿀 수 있습니다.
예: `width:=640 height:=400`으로 USB 전송과 프리뷰 변환량을 줄일 수 있습니다. 모델 텐서 크기는 선택한 모델에 따릅니다.

수신 스레드와 처리/GUI 루프를 분리하고 장치/호스트 큐를 모두 크기 1, non-blocking으로 설정합니다.
처리가 느리면 오래된 프레임을 쌓지 않고 최신 프레임을 사용합니다. 실제 입력·출력 FPS는 차량 PC에서 측정해야 합니다. 2초마다 평균 FPS와 추론 시간이 로그에 나옵니다.
입력 FPS는 센서 내부 FPS가 아닌 유효 프레임의 호스트 도착률입니다.

## 모델 전환 및 측정

`448x224 FP16` 버튼은 하단 80% ROI 전용 모델을 선택합니다. 1280×800 카메라 프레임에서
상단 160행을 제외한 영역을 448×224로 변환합니다. 이 모델은 검출 결과를 점수순 최대 100개로
표시하며, 측정 로그에 모델 입력 크기와 실제 ROI를 기록합니다. `FP16` 또는 `INT8`으로
돌아가면 512×320 모델에서 사용하던 ROI 설정이 적용됩니다.

시작할 때부터 448×224 모델을 쓰려면 아래처럼 실행합니다.

```bash
ros2 launch multiclass_yolo preview.launch.py model_variant:=roi80_448x224 engine_precision:=fp16
```

448×224 ONNX는 FP32 형식이며 Jetson에서 FP16 엔진으로 실행합니다. `INT8` 버튼은 동봉된
512×320 Q/DQ 모델을 선택합니다.

- `FP16`은 원본 holdout10 ONNX, `INT8`은 동봉된 Entropy Q/DQ ONNX를 사용합니다.
  두 모델은 입력·출력과 클래스 순서가 같고, INT8의 외부 입출력도 FP32입니다.
- 첫 선택 시 현재 Jetson에서 엔진을 생성하고 워밍업합니다. 전환 중 기존 엔진으로 추론을 계속하며,
  새 엔진이 준비되면 전환합니다. 이후 정밀도별로 엔진 캐시를 사용합니다.
  INT8 파일을 바꾸려면 `int8_model_path:=/absolute/model.int8.qdq.onnx`를 지정합니다.
- `Duration (s)`에서 1–120초를 입력하고 `Measure`를 누릅니다. 기본값은 15초입니다.
  `Stop`으로 일찍 끝낼 수 있습니다. 설정을 바꾸려면 현재 측정을 종료한 뒤 새 측정을 시작합니다.
- `Video off`에서는 추론을 계속하면서 표시용 색 변환, 박스·목록 갱신과 영상 확대를 건너뜁니다.
  같은 조건에서 `Video on`과 별도로 측정하면 프리뷰 비용을 비교할 수 있습니다.

측정 파일은 기본적으로 `~/.cache/multiclass_yolo/measurements/` 아래 실행별 폴더에 저장됩니다.
`measurement_directory`와 `measurement_seconds`로 경로와 초기 시간을 지정할 수 있습니다.
`frames.csv`는 프레임별 시간이며 해당하지 않는 단계는 빈 칸입니다. `summary.json`은 단계별 p50/p95,
수신·검출 완료·프리뷰 제출 FPS, 시퀀스 누락, 처리 루프가 건너뛴 프레임과 측정 설정을 담습니다.
`receive_to_submit_ms`는 호스트 수신부터 HighGUI 제출 완료까지이며 모니터 표시 시각과는 별개입니다.
시간 합계를 낼 때 `processing_ms`와 `receive_to_*`는 다른 구간을 포함합니다.

INT8 모델은 보정 스케일을 ONNX 안에 포함하고 있습니다. Jetson에서 같은 전력 모드·ROI·FPS로
`inference_ms`와 전체 검출 FPS를 함께 비교하세요. 레이어 정밀도를 확인할 때는 설치된 TensorRT의
`trtexec`로 Q/DQ 모델을 상세 프로파일링 옵션과 함께 별도로 빌드해 레이어 정보를 확인합니다.
동봉 모델의 기존 검출 평가는 전체 프레임 기준이므로 ROI 100%와 운용 ROI 65%에서 각각 확인합니다.

## 검증

```bash
colcon test --packages-select multiclass_yolo --event-handlers console_direct+
colcon test-result --verbose
```

후처리 테스트는 클래스별 NMS/임계값, 점수 계산, 잘못된 값, 원본 좌표 복원을 검증합니다.
모델 smoke test는 실제 ONNX CPU 실행, 유한한 `[1,3360,9]`, RGB/BGR 경로의 박스·점수 일치와 ROI 박스의 원본 좌표 복원을 검사합니다.
0 임계값으로 낮은 점수 후보도 남겨 경로 일치 검사가 빈 검출 결과만 비교하지 않도록 했습니다.

작성 시 Mac OpenCV 4.13에서 위 C++ 후처리/ONNX 테스트를 직접 컴파일하여 통과했습니다.
ROS launch Python 문법과 package.xml 파싱도 확인했습니다.
ROS 노드 전체 빌드, OAK 실시간 영상, TensorRT/CUDA 실행 및 실제 30 FPS는 이 Mac에 해당 환경이 없어 검증하지 못했습니다.

TensorRT 없이 CPU만 빌드할 때는 `-DMULTICLASS_YOLO_TENSORRT=OFF`를 사용하고,
실행 시 `input_format:=rgb inference_backend:=CPU engine_precision:=fp32`를 지정합니다.
카메라용 DepthAI SDK는 여전히 필요합니다.


### 창이 바로 닫히는 현상

일부 GTK OpenCV는 `WND_PROP_VISIBLE` 조회에 -1(미지원)을 반환한다.
이를 창 닫힘으로 해석하지 않도록, 처음 실제로 지원된 속성의 변화를 보고 종료한다.
GTK에서는 `WND_PROP_AUTOSIZE`도 창 존재 확인에 사용하며 WINDOW_NORMAL의 0은 정상이다.
첫 영상 전에는 대기 화면을 표시하고, 최초 수신·최초 검출 표시·종료 사유를 로그에 남긴다.
엔진 캐시는 유지되므로 GUI 수정만으로 엔진을 다시 만들 필요는 없다.
