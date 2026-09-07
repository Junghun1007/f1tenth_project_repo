# traffic_detection_test

`camera_driver`를 수정하지 않고 OAK 카메라의 신호등 검출 결과만 확인하는
ROS 2 C++ 프리뷰 패키지다. OAK CAM_A의 `1280x800` 센서 영상을 `640x400`
NV12로 받아 호스트에서 BGR로 변환하고, 학습 완료된 FP32 YOLOX-S ONNX
모델로 추론한다.

## 처리 방식

- 카메라 입력: `640x400`, 기본 80 FPS, 장치 왜곡 보정 사용
- 모델 입력: BGR FP32 `[1, 3, 640, 640]`
- 전처리: 영상은 늘리지 않고 좌측 상단에 유지하며 아래쪽 240행을 값 114로
  패딩
- 모델 출력: decoded `[1, 8400, 6]`
- confidence: `objectness * class probability`
- 기본 threshold: score `0.25`, NMS IoU `0.65`
- 실행 백엔드: OpenCV DNN CPU, FP32
- 출력: OpenCV 프리뷰 창만 사용하며 ROS 이미지나 검출 토픽은 발행하지 않음

캡처 스레드는 큐를 쌓지 않고 가장 최신 프레임만 보관한다. 추론 속도가
카메라 속도보다 낮으면 오래된 프레임을 건너뛰므로 프리뷰 지연이 계속
누적되지 않는다. 프리뷰에는 bounding box, confidence, 검출 개수와 추론
시간을 표시한다. `Q` 또는 `ESC`로 종료한다.

## 모델 범위

번들 모델은 학습할 때 `Red`와 `Green`을 단일 `traffic_light` 클래스로
합쳤다. 따라서 이 패키지는 신호등의 존재와 위치만 표시하며 빨간불과
초록불 상태를 구분하지 않는다. 양자화 및 TensorRT 변환도 포함하지 않는다.

기본 모델은 다음 설치 경로에서 자동으로 불러온다.

```text
share/traffic_detection_test/models/traffic_light_yolox_s_640_batch_1.onnx
```

`model_path` launch 인자로 다른 decoded YOLOX ONNX 파일을 지정할 수 있지만,
입출력 형식은 `[1,3,640,640]`과 `[1,N,6]`이어야 한다.

## 빌드

Ubuntu/Jetson의 ROS 2 Humble, DepthAI C++ 3.x와 OpenCV 4의 `dnn`,
`highgui` 모듈이 필요하다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select traffic_detection_test \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

DepthAI C++가 기본 prefix에 없다면 `camera_driver`와 같은 방식으로
`-Ddepthai_DIR=<prefix>/lib/cmake/depthai`를 전달한다.

## 실행

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py
```

threshold 변경 예시는 다음과 같다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  score_threshold:=0.30 nms_threshold:=0.65
```

이 패키지는 OAK 장치를 직접 연다. 같은 OAK 장치를 사용하는
`camera_driver` 또는 다른 카메라 패키지와 동시에 실행할 수 없다. GUI가
필수이므로 SSH에서 실행할 때는 X11 forwarding이나 로컬 디스플레이가
설정되어 있어야 한다.
