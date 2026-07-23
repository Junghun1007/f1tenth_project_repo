# image_processor

`normal_image_publisher_node`는 `/camera/image_raw`를 구독해
`/image/normal`로 변경 없이 재발행합니다. 왜곡 보정을 포함한 영상 생성은
`camera_driver`의 DepthAI 파이프라인에서 수행하며, 이 노드는 OpenCV/NPZ
보정을 하지 않습니다.

`normal_image.launch.py`는 `camera_driver_node`와
`normal_image_publisher_node`를 함께 실행합니다.

```bash
colcon build --packages-select camera_driver image_processor
source install/setup.bash
ros2 launch image_processor normal_image.launch.py
```

`image_processor/config/normal_image.yaml`에서 다음 값을 설정합니다.

- `undistort_enabled`: DepthAI 장치 내부 렌즈 보정 on/off
- `publish_rate_hz`: `/image/normal`의 최대 발행 Hz

발행률을 카메라 입력 FPS보다 높여도 프레임을 복제하지 않습니다.
