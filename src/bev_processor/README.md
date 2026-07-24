# bev_processor

`camera_driver`와 별도 프로세스로 동작하는 C++ ROS 2 BEV 노드다. 왜곡
보정된 `/camera/image_rect`의 최신 960x540 BGR8 프레임을 지면에 역투영해
`/camera/image_bev`로 발행하고 별도 OpenCV 창에 표시한다.

## 성능 설계

- 입력 콜백은 최신 메시지 포인터만 교체하고 즉시 반환한다.
- BEV 좌표의 역투영 LUT는 시작할 때 한 번 계산하고, 프레임마다
  `cv::remap`만 실행한다.
- 변환, ROS 발행, 프리뷰를 서로 다른 스레드로 분리한다.
- 처리량보다 입력이 빠르면 대기열을 쌓지 않고 오래된 프레임을 건너뛰어
  지연을 제한한다.
- 변환은 입력 이벤트마다 최대 속도로 실행한다. 프리뷰만 기본 30 FPS로
  제한하며 ROS 발행은 기본적으로 모든 변환 완료 프레임을 전달한다.

독립 프로세스이므로 처리 FPS는 `/camera/image_rect`가 실제로 들어오는
FPS를 넘을 수 없다. 예를 들어 `camera_driver`의 상태 로그에서 ROS가
87 Hz이면 BEV의 `input`과 `processed`도 최대 약 87 Hz다. BEV 로그는
입력 병목(`input`)과 변환 병목(`processed`, `skipped`)을 구분해 보여준다.

## 빌드

워크스페이스 루트에서 실행한다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 실행

터미널 1에서 카메라 토픽을 발행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=false \
  publish_enabled:=true
```

터미널 2에서 BEV 노드만 독립 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bev_processor bev_processor.launch.py
```

BEV 프리뷰를 끄거나 토픽 발행을 끄려면 다음처럼 덮어쓴다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=false \
  publish_enabled:=true
```

프리뷰만 60 FPS로 올리려면 `preview_max_fps:=60.0`을 추가한다.

`q`, `Q`, `Esc` 또는 창 닫기 버튼은 BEV 프리뷰만 닫는다. 변환과 ROS
발행은 계속된다. 상태 로그의 `processed`가 실제 BEV 변환 완료 FPS이고,
`skipped`는 처리 중 새 입력으로 교체되어 의도적으로 버린 오래된
프레임 수다.

## 현재 보정값

- 입력/K_rect: 960x540,
  `fx=421.050720215`, `fy=378.767059326`,
  `cx=482.274475098`, `cy=265.019256592`
- 카메라 높이 0.20 m, 하향각 14도, yaw 0도
- 전방 0.18~5.0 m, 좌우 -0.35~0.35 m
- 0.01 m/px, 출력 70x482

카메라 위치나 각도가 바뀌면
`config/bev_config.yaml`의 외부 파라미터를 다시 측정해야 한다.
