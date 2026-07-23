# image_viewer

`sensor_msgs/msg/Image` 토픽의 최신 프레임을 지정한 FPS로 표시하는 ROS 2
뷰어 패키지입니다. `ImageViewerNode` 클래스 하나를 서로 다른 노드 이름,
토픽, 프리뷰 FPS로 여러 번 실행하는 구조입니다.

구독 콜백은 최신 메시지만 보관하고 OpenCV 변환과 화면 표시는 프리뷰
타이머에서 수행합니다. 따라서 프리뷰 FPS를 낮춰도 원본 카메라 및 토픽
발행 FPS에는 영향을 주지 않습니다.

기본값인 `fit_to_image: true`에서는 입력 영상의 해상도에 맞춰 창 크기를
자동으로 변경합니다. 영상이 `max_window_width`, `max_window_height`보다
크면 종횡비를 유지하면서 창만 축소합니다. 원본 이미지 데이터는
리사이즈하지 않습니다. `fit_to_image: false`로 설정하면 `window_width`,
`window_height`의 고정 창 크기를 사용합니다.

## 단일 뷰어

```bash
ros2 launch image_viewer image_viewer.launch.py \
  node_name:=viewer_1 \
  image_topic:=/camera/image_raw \
  preview_fps:=15.0 \
  window_name:="camera raw" \
  fit_to_image:=true \
  max_window_width:=1280 \
  max_window_height:=720
```

다른 토픽을 보는 두 번째 인스턴스:

```bash
ros2 launch image_viewer image_viewer.launch.py \
  node_name:=viewer_2 \
  image_topic:=/image/normal \
  preview_fps:=10.0 \
  window_name:="normal"
```

## viewer_1~3 동시 실행

각 뷰어 설정은 `image_viewer/config/image_viewers.yaml`에서 변경합니다.

```bash
ros2 launch image_viewer image_viewers.launch.py
```

각 창에서 `Q`, `q`, `ESC`를 누르거나 창을 닫으면 해당 뷰어 노드만
종료됩니다. 토픽에서 이미지가 들어오지 않으면 주기적으로 경고 로그를
출력합니다.

OpenCV GUI를 사용하므로 모니터가 연결되어 있거나 SSH X11 forwarding 등으로
`DISPLAY` 또는 `WAYLAND_DISPLAY`가 설정된 환경에서 실행해야 합니다.
