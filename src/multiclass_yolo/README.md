# multiclass_yolo

OAK CAM_A 영상을 받아 YOLOX 4개 클래스의 검출 결과를 보는 독립 ROS 2 프리뷰 패키지입니다.
기존 주행/신호등 패키지는 수정하지 않습니다. 이 노드는 OAK 장치를 직접 열므로 같은 장치를 사용하는
`camera_driver`, `bev_processor`, `traffic_detection_test` 등은 종료한 후 실행하세요.

## 모델 계약

기본 모델은 `multiclass_yolox_tiny_512x320_holdout10.onnx`입니다.
동봉된 `.metadata.json`과 `.labels.txt`에 입력·출력 형식과 클래스 순서가 기록되어 있습니다.

- 입력: FP32 `[1,3,320,512]`, **BGR**, 0–255, 정규화 없음
- 리사이즈: 종횡비 유지, 좌상단 배치, 오른쪽/아래를 114로 채움
- 출력: FP32 `detections [1,3360,9]`
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

- 영상과 클래스명·점수·박스
- `AVG input`: 첫 호스트 수신부터 현재까지의 평균 수신 FPS `(수신 수 - 1) / 경과시간`
- `AVG output`: 같은 시점부터 표시를 완료한 검출 프레임 수 / 경과시간
- 처음 0.5초는 FPS를 0으로 표시하여 초기값 급등을 피함. 엔진 빌드/카메라 시작 대기는 제외
- `total`: 현재 프레임의 변환·전처리·추론·후처리·박스 그리기 시간 (GUI 표시 시간 제외)
- `convert`: RGB 경로는 추론 전 NV12→RGB 시간, NV12 경로는 추론 후 프리뷰 NV12→BGR 시간
- `skipped`: 호스트 수신 후 최신 프레임 우선 처리 때문에 검출하지 않은 프레임 수 (장치/USB 드롭 수와 다름)
- 4개 슬라이더: `con`, `truck`, `sedan`, `traffic_light`. 0–100을 0.00–1.00 점수로 변환하여 다음 추론에 적용
- Q / ESC / 창 닫기로 종료 (창 상태 조회 미지원 백엔드에서는 Q / ESC 또는 Ctrl+C 사용)

슬라이더 값은 현재 실행에만 적용됩니다. 재실행 때 유지하려면 실행 인수 또는 YAML을 수정하세요.
YAML로 실행하려면 아래처럼 `ros2 run`에 전달합니다 (기본 launch는 launch 인수를 사용합니다).

```bash
ros2 run multiclass_yolo multiclass_yolo_node --ros-args \
  --params-file src/multiclass_yolo/config/preview.yaml
```

카메라 기본값은 CAM_A, 1280×800, 30 FPS, 전체 화면, 기존 프로젝트처럼 undistort 활성화입니다.
`width`, `height`, `sensor_fps`, `undistort_enabled`를 launch 인수로 바꿀 수 있습니다.
예: `width:=640 height:=400`으로 USB 전송과 프리뷰 변환량을 줄일 수 있습니다. 모델 텐서는 계속 512×320입니다.

수신 스레드와 처리/GUI 루프를 분리하고 장치/호스트 큐를 모두 크기 1, non-blocking으로 설정합니다.
처리가 느리면 오래된 프레임을 쌓지 않고 최신 프레임을 사용합니다. 입력 목표는 30 FPS이며
**실제 입력/출력 30 FPS 달성 여부는 차량 PC에서 측정해야 합니다.** 2초마다 평균 FPS와 추론 시간이 로그에 나옵니다.
입력 FPS는 센서 내부 FPS가 아닌 유효 프레임의 호스트 도착률입니다.

## 검증

```bash
colcon test --packages-select multiclass_yolo --event-handlers console_direct+
colcon test-result --verbose
```

후처리 테스트는 클래스별 NMS/임계값, 점수 계산, 잘못된 값, 원본 좌표 복원을 검증합니다.
모델 smoke test는 실제 ONNX CPU 실행, 유한한 `[1,3360,9]`, RGB/BGR 경로의 박스·점수 일치를 검사합니다.
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
