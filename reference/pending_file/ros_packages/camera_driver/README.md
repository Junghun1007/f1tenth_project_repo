# camera_driver

OAK/DepthAI 카메라를 열고 원본 프레임 수신 여부를 확인하는 ROS 2
패키지입니다.

이 노드는 전용 수신 스레드에서 OAK 프레임을 계속 가져와 최신 프레임을
유지합니다. ROS 발행도 별도 스레드로 분리하며, OpenCV 프리뷰는 최신
프레임을 직접 표시합니다. 따라서 각각의 FPS를 별도로 설정할 수 있습니다.
차선 검출 등의 호스트 영상 처리는 하지 않습니다.

`/camera/image_raw`의 형식은 JPG가 아닌 비압축 `bgr8` 픽셀 데이터입니다.
메시지의 `encoding`은 `bgr8`, `step`은 `width * 3`이며 픽셀 바이트가
`data` 필드에 들어갑니다. 여기서 `raw`는 비압축 ROS 이미지라는 의미이고,
카메라 센서의 Bayer RAW 형식을 의미하지는 않습니다.

## 준비

Jetson의 ROS 2 환경에 DepthAI Python 패키지를 설치합니다.

```bash
python3 -m pip install depthai
```

카메라 구동 설정은
`camera_driver/config/camera_config.yaml`에서 변경합니다. `enabled: true`이면
노드 시작 시 `CAM_A`의 DepthAI 파이프라인을 시작하고, `false`이면 카메라를
열지 않습니다.

`normal_image.launch.py`로 전체 영상 입력을 실행할 때는
`image_processor/config/normal_image.yaml`의 `undistort_enabled`가 DepthAI
`Camera.requestOutput()`의 `enableUndistortion` 옵션에 직접 전달되고,
카메라 드라이버의 출력 토픽은 `/image/normal`로 변경됩니다.

- `false`: 렌즈 왜곡 보정 없이 출력
- `true`: OAK 장치 내부 캘리브레이션을 사용하는 DepthAI 왜곡 보정 출력

별도 NPZ 파일이나 OpenCV 카메라 행렬은 사용하지 않습니다.

`sensor_fps`는 DepthAI 카메라가 프레임을 취득하는 FPS입니다. 카메라
수신 스레드는 이 속도로 최신 프레임을 갱신합니다. `publish_fps`는 ROS
이미지 발행의 최대 속도이고 `preview_fps`는 ROS 메시지 변환 없이
`cv2.imshow`에 전달하는 직접 프리뷰 속도입니다. 발행 또는 프리뷰가
카메라보다 빠르게 설정되어도 프레임을 복제하지 않습니다.

ROS 2 Humble의 Python `uint8[]` 필드가 일반 바이트열을 원소별로 검사하는
병목을 피하기 위해 이미지 데이터는 `array.array('B')` 빠른 경로로
설정합니다.

주요 실행 파라미터:

- `publish_enabled`: ROS 이미지 토픽 발행 on/off
- `publish_fps`: ROS 이미지 최대 발행 FPS
- `preview_enabled`: DepthAI 프레임 직접 프리뷰 on/off
- `preview_fps`: 직접 프리뷰 최대 FPS
- `preview_window_name`: 직접 프리뷰 창 이름

`normal_image.launch.py`에서는 같은 드라이버가 중간 재발행 노드 없이
왜곡 보정된 영상을 `/image/normal`로 직접 발행하고 직접 프리뷰합니다.

`camera_height_m`과 `camera_downward_angle_deg`는 임의 초기값으로
YAML에 기록되어 있으며 시작 로그에 표시됩니다. 현재 영상 처리에는
사용하지 않습니다.

## 실행

```bash
colcon build --packages-select camera_driver
source install/setup.bash
ros2 launch camera_driver camera_driver.launch.py
```

정상적으로 프레임을 받으면 다음 로그가 출력됩니다.

```text
Camera frame reception verified: 640x480, encoding=bgr8, DepthAI format=BGR888i
```

상태 로그는 카메라 수신, ROS 발행, 직접 프리뷰를 구분합니다.

```text
Camera active: capture=100.0Hz/..., ROS_publish=..., direct_preview=...
```

발행 토픽 확인:

```bash
ros2 topic hz /camera/image_raw
ros2 topic info /camera/image_raw
```
