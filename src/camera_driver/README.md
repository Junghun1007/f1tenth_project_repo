# camera_driver

OAK/DepthAI 카메라 영상을 낮은 지연시간으로 받는 ROS 2 C++ 패키지다.
기본 설정은 다음과 같다.

- 출력: `640x480`, `bgr8`
- 요청 센서 FPS: `120`
- 렌즈 왜곡 보정: OAK 장치 내부에서 활성화
- ROS 토픽: `/camera/image_rect`
- QoS: sensor data, best effort, keep-last 1
- 호스트 큐: 크기 2, non-blocking
- 프리뷰: 캡처와 분리된 최신 프레임 방식

`sensor_fps: 120.0`은 센서에 요청하는 값이다. 실제 OAK 모델의 `CAM_A`
센서가 선택한 해상도에서 120 FPS를 지원해야 실제 수신 속도도 120 FPS가
된다. 노드는 요청값과 별도로 측정된 캡처 FPS와 장치 sequence gap을
주기적으로 출력한다.

## 성능 구조

카메라 캡처, ROS 발행, OpenCV 프리뷰는 서로 다른 스레드에서 실행된다.
캡처 스레드는 DepthAI 큐를 비우고 최신 프레임 포인터만 교체하며, ROS
메시지 복사나 `imshow()`를 수행하지 않는다. 발행이나 프리뷰가 늦어지면
오래된 프레임을 쌓지 않고 최신 프레임으로 건너뛴다.

왜곡 보정은 `Camera::requestOutput(..., enableUndistortion=true)`로 요청한다.
따라서 호스트에서 `cv::remap()`을 수행하지 않는다.

ROS 발행은 `sensor_msgs/msg/Image`의 `UniquePtr`를 사용한다. 기본 launch는
컴포넌트 컨테이너에서 intra-process 통신을 활성화한다. 향후 C++ 영상 처리
컴포넌트를 같은 컨테이너에 적재하면 DDS 직렬화 없이 메시지 소유권을 넘길
수 있다. 별도 프로세스의 구독자, `ros2 topic hz`, rosbag 등은 DDS 전송과
추가 메모리 복사를 사용한다.

프리뷰 창의 실제 표시 속도는 모니터 주사율과 OpenCV GUI 성능의 제한을
받는다. 60 Hz 모니터에서는 센서가 120 FPS로 동작해도 120개의 서로 다른
프레임을 모두 눈으로 확인할 수 없다. 상태 로그의 `capture` 값이 센서
수신 속도의 기준이다.

## 요구 사항

- Ubuntu/Jetson의 ROS 2 Humble
- DepthAI C++ 3.x
- OpenCV 4
- OAK 장치와 USB 3 연결

DepthAI를 별도 prefix에 설치했다면 빌드 전에 경로를 지정한다.

```bash
export CMAKE_PREFIX_PATH=/path/to/depthai-install:$CMAKE_PREFIX_PATH
```

DepthAI는 OpenCV 지원을 켜고 빌드되어야 한다. 이 패키지는
`ImgFrame::getCvFrame()`과 OpenCV 프리뷰를 사용한다.

## 빌드

Jetson에서 워크스페이스 루트로 이동한 후 Release 모드로 빌드한다.

```bash
colcon build \
  --packages-select camera_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 실행

기본 컴포넌트 실행:

```bash
ros2 launch camera_driver camera_driver.launch.py
```

창 없이 실행:

```bash
ros2 launch camera_driver camera_driver.launch.py preview_enabled:=false
```

ROS 이미지 발행 없이 캡처와 직접 프리뷰만 측정:

```bash
ros2 launch camera_driver camera_driver.launch.py publish_enabled:=false
```

독립 실행 파일도 제공한다. 이 실행 파일 역시 intra-process 옵션을 켠다.

```bash
ros2 run camera_driver camera_driver_node \
  --ros-args --params-file \
  "$(ros2 pkg prefix camera_driver)/share/camera_driver/config/camera_config.yaml"
```

## 상태 확인

노드는 기본 5초마다 다음 항목을 출력한다.

- `capture`: 실제 DepthAI 프레임 수신 FPS와 누적 프레임 수
- `device_gap`: DepthAI sequence 번호로 계산한 누락 프레임 수
- `ROS`: 실제 ROS 이미지 발행 FPS
- `preview`: 실제 프리뷰 갱신 FPS
- `latest_age`: 최신 센서 프레임의 현재 나이
- `errors`: 캡처, 발행, 잘못된 프레임 오류 수

예:

```text
Camera status: capture=119.8/120.0 Hz (...), device_gap=0/0,
ROS=119.7 Hz (...), preview=59.9 Hz (...), latest_age=3.20 ms, ...
```

외부 토픽 확인은 다음과 같이 할 수 있다. 단, 이 명령 자체가 별도 DDS
구독자를 추가하므로 최종 성능 판정은 드라이버의 `capture` 로그를 우선한다.

```bash
ros2 topic hz /camera/image_rect
ros2 topic info /camera/image_rect --verbose
```

## 주요 파라미터

설정 파일은 `config/camera_config.yaml`이다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `sensor_fps` | `120.0` | OAK 센서/출력 요청 FPS |
| `width`, `height` | `640`, `480` | 출력 해상도 |
| `undistort_enabled` | `true` | OAK 장치 내부 왜곡 보정 |
| `queue_size` | `2` | DepthAI 호스트 큐 크기 |
| `queue_blocking` | `false` | 큐가 찼을 때 캡처 차단 여부 |
| `publish_enabled` | `true` | ROS 이미지 발행 |
| `publish_fps` | `120.0` | ROS 발행 최대 FPS |
| `preview_enabled` | `true` | OpenCV 직접 프리뷰 |
| `preview_fps` | `120.0` | 프리뷰 갱신 최대 FPS |

120 FPS에서 `640x480 bgr8` 영상 데이터는 메시지 헤더와 DDS 오버헤드를
제외하고도 약 105 MiB/s다. 외부 노드가 전체 이미지를 항상 필요로 하지
않는다면 `publish_enabled: false`를 사용하거나 발행 FPS를 낮추고, 주 영상
처리는 같은 프로세스의 C++ 컴포넌트로 구성하는 것이 좋다.
