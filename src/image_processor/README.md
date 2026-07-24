# image_processor

`normal_image.launch.py`는 `camera_driver_node`를 실행하고 DepthAI에서
생성한 영상을 `/image/normal`로 직접 발행합니다. 변경 없는 대용량 영상을
Python 노드에서 다시 발행하지 않으므로 `/camera/image_raw`에서
`/image/normal`로 이어지던 중간 복사와 ROS 직렬화 과정이 없습니다.

왜곡 보정은 `camera_driver`의 DepthAI 파이프라인에서 수행하며 별도
OpenCV/NPZ 보정을 하지 않습니다. 이전 `normal_image_publisher_node`
실행 파일은 호환성을 위해 남아 있지만 기본 launch에서는 실행하지 않습니다.

```bash
colcon build --packages-select camera_driver image_processor
source install/setup.bash
ros2 launch image_processor normal_image.launch.py
```

이 명령은 카메라와 `Normal rectified image` 창만 실행합니다. 프리뷰 없이
`/image/normal`만 발행하려면 다음처럼 실행합니다.

```bash
ros2 launch image_processor normal_image.launch.py preview_enabled:=false
```

`image_processor/config/normal_image.yaml`에서 다음 값을 설정합니다.

- `undistort_enabled`: DepthAI 장치 내부 렌즈 보정 on/off
- `image_topic`: 카메라 드라이버의 직접 출력 토픽
- `preview_enabled`: 왜곡 보정 영상 직접 프리뷰 on/off
- `preview_fps`: 왜곡 보정 영상 직접 프리뷰 FPS

센서 취득 FPS와 ROS 발행 FPS는
`camera_driver/config/camera_config.yaml`의 `sensor_fps`,
`publish_fps`로 각각 설정합니다. 프리뷰는 `/image/normal`을 다시
구독하지 않고 DepthAI의 왜곡 보정 결과를 카메라 노드에서 바로 표시합니다.
