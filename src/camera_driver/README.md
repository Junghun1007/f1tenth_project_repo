# camera_driver

OAK/DepthAI 카메라 영상을 낮은 지연시간으로 받는 ROS 2 C++ 패키지다.
기본 설정은 다음과 같다.

- 출력: `960x540`, `bgr8`
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

첫 프레임에는 최종 출력에 적용된 resize와 장치 내부 왜곡 보정을 모두
반영한 `K_rect`의 `fx`, `fy`, `cx`, `cy`도 한 번 출력한다. BEV처럼
투영 기하가 필요한 후속 처리에서는 원본 센서 행렬 대신 이 값을 사용한다.

## 성능 구조

카메라 캡처, ROS 발행, OpenCV 프리뷰는 서로 다른 스레드에서 실행된다.
캡처 스레드는 DepthAI 큐를 비우고 최신 프레임 포인터만 교체하며, ROS
메시지 복사나 `imshow()`를 수행하지 않는다. 발행이나 프리뷰가 늦어지면
오래된 프레임을 쌓지 않고 최신 프레임으로 건너뛴다.

OAK가 전달한 `BGR888i` 버퍼는 캡처 스레드에서 다시 복사하지 않는다.
`cv::Mat`은 패킷 메모리를 가리키는 zero-copy 뷰이고, 최신 프레임
스냅샷이 DepthAI 패킷의 수명을 함께 유지한다. ROS 토픽 발행을 선택한
경우에만 `sensor_msgs/Image` 데이터로 한 번 복사한다.

왜곡 보정은 `Camera::requestOutput(..., enableUndistortion=true)`로 요청한다.
따라서 호스트에서 `cv::remap()`을 수행하지 않는다.

ROS 발행은 `sensor_msgs/msg/Image`의 `UniquePtr`를 사용한다. 기본 launch는
컴포넌트 컨테이너에서 intra-process 통신을 활성화한다. 향후 C++ 영상 처리
컴포넌트를 같은 컨테이너에 적재하면 DDS 직렬화 없이 메시지 소유권을 넘길
수 있다. 별도 프로세스의 구독자, `ros2 topic hz`, rosbag 등은 DDS 전송과
추가 메모리 복사를 사용한다.

`publish_fps`가 센서 FPS 이상이면 고정 주기의 타이머로 최신 영상을
샘플링하지 않고 새 프레임 도착 알림에 맞춰 발행한다. 두 120 Hz 주기의
미세한 위상 차이로 프레임을 건너뛰는 현상을 피하기 위한 동작이다.

프리뷰 창의 실제 표시 속도는 모니터 주사율과 OpenCV GUI 성능의 제한을
받는다. 60 Hz 모니터에서는 센서가 120 FPS로 동작해도 120개의 서로 다른
프레임을 모두 눈으로 확인할 수 없다. 상태 로그의 `capture` 값이 센서
수신 속도의 기준이다.

## 요구 사항

- Ubuntu/Jetson의 ROS 2 Humble
- DepthAI C++ 3.x
- OpenCV 4
- OAK 장치와 USB 3 연결

Python의 `pip install depthai`만으로는 이 C++ 패키지를 빌드할 수 없다.
`depthaiConfig.cmake`와 `depthai::core` 공유 라이브러리를 제공하는 DepthAI
C++ 설치가 필요하다.

### Jetson에 DepthAI C++ 설치

ROS 2 Humble을 사용하는 Ubuntu/Jetson에서 필요한 기본 패키지를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  build-essential git cmake libudev-dev libopencv-dev
```

DepthAI C++ 3.x 소스를 받아 공유 라이브러리로 빌드한다. 메모리가 부족한
Jetson을 고려해 병렬 빌드 수는 2로 제한한다.

```bash
cd ~/Desktop/f1tenth_test0724
git clone \
  --branch v3.6.1 \
  --depth 1 \
  --recurse-submodules \
  https://github.com/luxonis/depthai-core.git

cmake \
  -S depthai-core \
  -B depthai-core/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DDEPTHAI_OPENCV_SUPPORT=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build depthai-core/build --parallel 2
sudo cmake --install depthai-core/build
sudo ldconfig
```

설치 결과를 확인한다.

```bash
find /usr/local -name depthaiConfig.cmake -print
```

일반적으로 다음 경로가 출력된다.

```text
/usr/local/lib/cmake/depthai/depthaiConfig.cmake
```

다른 prefix에 설치했다면 워크스페이스 빌드 시 해당 위치를 직접 전달한다.

```bash
colcon build \
  --packages-select camera_driver \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Ddepthai_DIR=/path/to/depthai-install/lib/cmake/depthai
```

DepthAI는 OpenCV 지원을 켜고 빌드되어야 한다. 이 패키지는
`ImgFrame::getFrame()`의 zero-copy OpenCV 뷰와 OpenCV 프리뷰를 사용한다.

## 빌드

Jetson에서 워크스페이스 루트로 이동한 후 Release 모드로 빌드한다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select camera_driver \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`colcon build`는 `f1tenth_project_repo` 루트에서 실행한다. `src` 안에서
실행하면 그 아래에 별도의 `build`, `install`, `log`가 생성되어 올바른
워크스페이스의 설치 결과와 섞일 수 있다.

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
| `width`, `height` | `960`, `540` | 출력 해상도 |
| `undistort_enabled` | `true` | OAK 장치 내부 왜곡 보정 |
| `queue_size` | `2` | DepthAI 호스트 큐 크기 |
| `queue_blocking` | `false` | 큐가 찼을 때 캡처 차단 여부 |
| `publish_enabled` | `true` | ROS 이미지 발행 |
| `publish_fps` | `120.0` | ROS 발행 최대 FPS |
| `preview_enabled` | `true` | OpenCV 직접 프리뷰 |
| `preview_fps` | `120.0` | 프리뷰 갱신 최대 FPS |

120 FPS에서 `960x540 bgr8` 영상 데이터는 메시지 헤더와 DDS 오버헤드를
제외하고도 약 178 MiB/s다. 외부 노드가 전체 이미지를 항상 필요로 하지
않는다면 `publish_enabled: false`를 사용하거나 발행 FPS를 낮추고, 주 영상
처리는 같은 프로세스의 C++ 컴포넌트로 구성하는 것이 좋다.
