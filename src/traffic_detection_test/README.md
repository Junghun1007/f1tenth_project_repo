# traffic_detection_test

`camera_driver`를 수정하지 않고 OAK 카메라의 신호등 검출 결과만 확인하는
ROS 2 C++ 프리뷰 패키지다. OAK CAM_A의 `1280x800` 센서 영상을 `640x400`
NV12로 받아 TensorRT 입력용 전처리를 GPU에서 수행하고, 학습 완료된
FP32 YOLOX-S ONNX 모델로 추론한다.

## 처리 방식

- 카메라 입력: `640x400`, 기본 80 FPS, 장치 왜곡 보정 사용
- 추론 ROI: 원본 픽셀 기준 중심 좌표와 폭·높이로 지정
- 기본 모델 입력: BGR FP32 `[1, 3, 160, 640]`
- TensorRT 전처리: raw NV12를 GPU로 전송한 뒤 CUDA 커널 하나로
  ROI crop·resize, `NV12 -> BGR FP32 NCHW`와 114 패딩을 수행
- CPU 전처리: 호스트 BGR에서 동일한 패딩과 FP32 NCHW 생성
- 기본 모델 출력: decoded `[1, 2100, 6]`
- confidence: `objectness * class probability`
- 기본 threshold: score `0.25`, NMS IoU `0.65`
- 기본 실행 백엔드: TensorRT 직접 실행, FP32
- 출력: OpenCV 프리뷰 창만 사용하며 ROS 이미지나 검출 토픽은 발행하지 않음

캡처 스레드는 큐를 쌓지 않고 가장 최신 프레임만 보관한다. 추론 속도가
카메라 속도보다 낮으면 오래된 프레임을 건너뛰므로 프리뷰 지연이 계속
누적되지 않는다. 프리뷰에는 bounding box, confidence, 검출 개수와 추론
시간을 표시한다. `Q` 또는 `ESC`로 종료한다.

TensorRT가 사용되면 프리뷰 표시를 위한 `getCvFrame()` BGR 변환은 추론이
완료된 뒤 별도로 수행된다. 이 BGR 프레임은 화면 표시에만 쓰이며
TensorRT 입력으로 다시 복사되지 않는다.
프리뷰 캔버스는 항상 원본 `640x400`을 유지하며, ROI 밖은 검정색으로
마스크하고 ROI 경계를 노란색으로 표시한다. 검출 상태 글자는 공간이
있으면 ROI 바로 아래의 검정 영역에 표시하므로 추론 영역을 가리지 않는다.

## 추론 ROI와 모델 크기

ROI는 `roi_center_x`, `roi_center_y`, `roi_width`, `roi_height`로 지정하며
좌표와 크기는 모두 `640x400` 원본 영상 픽셀 기준이다. ROI는
반드시 원본 영상 안에 완전히 들어와야 한다.

기본 번들 모델은 기존 `640x640` 체크포인트를 재학습 없이 고정
`640x160` 입력으로 다시 export한 모델이다. 기본 ROI는 중심 `(320,145)`,
크기 `640x160`, 즉 원본의 `y=65..224`이다. 목표 구간 `y=100..189`
주변에 위아래 약 35픽셀 문맥을 남기면서 TensorRT 입력 면적을 기존의
25%로 줄인다.

기본 설정은 별도 모델·ROI 인자 없이 실행할 수 있다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py
```

다른 고정 입력 ONNX를 사용하려면 `model_input_width`와
`model_input_height`를 실제 ONNX 입력과 같게 설정해야 하며 둘 다 32의
배수여야 한다.

ROI 비율과 모델 입력 비율이 다르면 원본 비율을 유지하여 좌측 상단에
resize하고 나머지를 114로 패딩한다. 검출 박스는 ROI offset을 다시
더해 원본 `640x400` 프리뷰 좌표로 복원한다.

상태 로그는 다음 구간의 평균과 최댓값을 각각 분리한다.

- 카메라 노출 중간 시점부터 Jetson 수신까지 `sensor->host`
- 화면 표시용 DepthAI 프레임의 BGR 변환 `preview-convert`
- TensorRT의 GPU NV12 변환·패딩·FP32 NCHW 생성, 또는 CPU의 호스트
  전처리 `preprocess`
- TensorRT는 host raw NV12, 기존 호스트 경로는 FP32 tensor의 GPU 입력
  전송 `H2D`
- TensorRT의 순수 network 실행 `execute`
- decoded FP32 출력의 host 전송 `D2H`
- score, 좌표 복원과 NMS 후처리
- bounding box와 상태 overlay 그리기
- host 수신 및 sensor 시점부터 `imshow()` 호출까지의 전체 지연

`sensor->display`는 `imshow()`에 프레임을 전달한 시점까지이며 모니터의 실제
화면 주사 완료 시각은 포함하지 않는다.

## 모델 범위

번들 모델은 학습할 때 `Red`와 `Green`을 단일 `traffic_light` 클래스로
합쳤다. 따라서 이 패키지는 신호등의 존재와 위치만 표시하며 빨간불과
초록불 상태를 구분하지 않는다. 기본 TensorRT 경로는 FP16, INT8, TF32 플래그를
모두 끄고 FP32 엔진만 생성한다. 양자화는 포함하지 않는다.

기본 모델은 다음 설치 경로에서 자동으로 불러온다.

```text
share/traffic_detection_test/models/traffic_light_yolox_s_640x160_batch_1.onnx
```

`model_path` launch 인자로 다른 decoded YOLOX ONNX 파일을 지정할 수 있지만,
입출력 형식은 FP32 `[1,3,H,W]`과 `[1,N,6]`이어야 한다.

TensorRT는 첫 실행에서 ONNX를 현재 Jetson용 FP32 엔진으로 빌드한다. 이 작업은
몇 분 걸릴 수 있으며, 다음 실행부터는 캐시된 엔진을 역직렬화해 바로 사용한다.
기본 캐시 파일은 ONNX 옆에 TensorRT major 버전을 포함한 다음 형식으로 생성된다.

```text
traffic_light_yolox_s_640x160_batch_1.onnx.trt<major>.fp32.engine
```

ONNX 파일이 캐시보다 새롭거나 캐시가 현재 TensorRT/GPU와 호환되지 않으면 자동으로
다시 빌드한다. `engine_cache_path`로 별도 위치를 지정할 수 있다. TensorRT 엔진은
Jetson GPU와 TensorRT 버전에 종속되므로 다른 장비에서 만든 파일을 복사해 쓰지 않는다.

## 빌드

Ubuntu/Jetson의 ROS 2 Humble, DepthAI C++ 3.x, OpenCV 4의 `dnn`,
`highgui` 모듈, CUDA Toolkit과 TensorRT 개발 패키지가 필요하다. OpenCV 자체는
CUDA 빌드일 필요가 없다. JetPack에 개발 패키지가 빠져 있다면 다음 라이브러리를
설치해야 한다.

```bash
sudo apt install libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev
```

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select traffic_detection_test \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/Desktop/0906ML/f1tenth_project_repo/install/setup.bash
```

DepthAI C++가 기본 prefix에 없다면 `camera_driver`와 같은 방식으로
`-Ddepthai_DIR=<prefix>/lib/cmake/depthai`를 전달한다.

이 프리뷰만 확인할 때는 위 명령처럼 `traffic_detection_test`만 선택한다.
`vehicle_bringup`을 `--packages-select`에 함께 넣으면 `auto_control`,
`joy_initializer`, `vehicle_dynamics_monitor` 등 아직 설치되지 않은 실행
의존성이 자동으로 추가되지 않아 전체 빌드가 중단될 수 있다. 전체 주행
패키지까지 새로 빌드해야 한다면 의존성을 포함하는 다음 명령을 사용한다.

```bash
colcon build \
  --packages-up-to traffic_detection_test vehicle_bringup \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
source ~/Desktop/0906ML/f1tenth_project_repo/install/setup.bash
```

`traffic_detection_test`는 NV12 전처리 CUDA 커널을 `.cu`로 직접 컴파일하므로
`nvcc`가 필요하다. 자동 탐색이 되지 않는 환경에서는 위 명령처럼
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 지정한다.

## 실행

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py
```

`ros2 launch`가 `/opt/ros/humble`에서만 패키지를 검색하며
`Package 'traffic_detection_test' not found`를 출력하면, 현재 터미널에서
아직 `install/setup.bash`를 source하지 않았거나 앞선 빌드가 설치 단계 전에
중단된 상태다. 위의 단독 빌드와 source 명령을 다시 실행한다.

threshold 변경 예시는 다음과 같다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  score_threshold:=0.30 nms_threshold:=0.65
```

기존 OpenCV CPU 기준과 비교할 때만 다음처럼 명시한다. `CPU`는 TensorRT 엔진
캐시를 생성하거나 사용하지 않는다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  inference_backend:=CPU
```

이 패키지는 OAK 장치를 직접 연다. 같은 OAK 장치를 사용하는
`camera_driver` 또는 다른 카메라 패키지와 동시에 실행할 수 없다. GUI가
필수이므로 SSH에서 실행할 때는 X11 forwarding이나 로컬 디스플레이가
설정되어 있어야 한다.
