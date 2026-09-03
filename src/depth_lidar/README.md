# depth_lidar

OAK 계열 스테레오 Depth 카메라를 직접 열어, 선택한 영상 ROI를 2D 라이다 형태의
`sensor_msgs/LaserScan`으로 변환하는 독립 ROS 2 테스트 패키지입니다. 다른 주행 노드나
카메라 드라이버를 사용하지 않습니다.

## 출력

- `~/scan` (`/depth_lidar/scan`): x축 전방, y축 좌측인 `LaserScan`
- `~/preview` (`/depth_lidar/preview`): 흰 배경, 빨간 점, 거리 원과 방사형 축을 표시한 Top-view
- 터미널: 처리 FPS, 카메라 촬영 시각부터 출력 완료까지의 delay, 순수 처리 시간

프리뷰 창이 필요하면 `preview.gui:=true`를 사용합니다. 기본값은 화면이 없는 Jetson에서도
동작하도록 이미지 토픽만 발행합니다.

## 빌드 및 실행

DepthAI C++ 라이브러리, OpenCV 및 ROS 2가 설치되어 있어야 합니다. OAK 장치는 이 노드가
직접 점유하므로 같은 장치를 쓰는 다른 카메라 노드를 먼저 종료합니다.

```bash
cd <workspace>
colcon build --packages-select depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py
```

RViz2에서는 `/depth_lidar/scan`을 `LaserScan`으로 추가합니다. 프리뷰는 다음처럼 확인할 수
있습니다.

```bash
ros2 run rqt_image_view rqt_image_view /depth_lidar/preview
```

## ROI 정의

ROI는 해상도와 무관한 비율로 계산됩니다.

```text
ROI 폭       = 영상 폭 × roi.width_ratio
ROI 높이     = 영상 높이 × roi.height_ratio
ROI 아래 경계 = 영상 아래에서 roi.bottom_offset_ratio 만큼 위
```

가로 ROI는 항상 화면 중앙에 놓입니다. 예를 들어 높이 비율 `0.10`, 아래 오프셋 `0.35`이면
ROI는 화면 아래에서 35% 위 지점부터 위쪽으로 영상 높이의 10%를 차지합니다.
`roi.height_ratio + roi.bottom_offset_ratio`는 1.0 이하여야 합니다.

## 실행 중 파라미터 변경

모든 실험 파라미터는 `ros2 param set`으로 실행 중 변경할 수 있습니다.

```bash
ros2 param set /depth_lidar roi.width_ratio 0.8
ros2 param set /depth_lidar roi.height_ratio 0.06
ros2 param set /depth_lidar roi.bottom_offset_ratio 0.40
ros2 param set /depth_lidar range.max_m 6.0
ros2 param set /depth_lidar preview.fps 5.0
ros2 param set /depth_lidar depth.mode high_accuracy
ros2 param set /depth_lidar camera.resolution 720p
ros2 param set /depth_lidar camera.fps 30.0
```

ROI, 거리, 스캔, 프리뷰 파라미터는 다음 프레임부터 즉시 반영됩니다. 해상도, 카메라 FPS,
Depth 모드 및 필터 파라미터는 장치 파이프라인을 자동으로 재시작한 뒤 반영됩니다.

| 파라미터 | 의미 |
|---|---|
| `camera.resolution` | `400p`, `480p`, `720p`, `800p` |
| `camera.fps` | 카메라 요청 FPS, 장치/해상도가 지원하는 범위에서 사용 |
| `depth.mode` | `default`, `high_density`, `high_accuracy` |
| `depth.confidence_threshold` | StereoDepth confidence 기준, 0~255 |
| `depth.left_right_check` | 좌우 일치 검사 |
| `depth.subpixel` | 원거리 정밀도 향상 모드 |
| `depth.extended_disparity` | 근거리 범위 확장; subpixel과 동시 사용 불가 |
| `depth.median_filter` | `off`, `3x3`, `5x5`, `7x7` |
| `roi.width_ratio` | 중앙 ROI 가로 비율 |
| `roi.height_ratio` | ROI 세로 비율 |
| `roi.bottom_offset_ratio` | 화면 하단에서 ROI 아래 경계까지의 비율 |
| `range.min_m`, `range.max_m` | 유효 깊이 범위 |
| `scan.bins` | LaserScan 각도 bin 개수 |
| `scan.pixel_stride` | ROI 픽셀 샘플 간격; 클수록 빠르지만 성긴 결과 |
| `scan.min_points_per_bin` | bin을 유효하게 만드는 최소 픽셀 수 |
| `preview.enabled`, `preview.gui` | 프리뷰 토픽/GUI 사용 여부 |
| `preview.fps`, `preview.size_px` | 프리뷰 갱신률과 정사각 이미지 크기 |

## 저지연 설정 팁

- DepthAI 출력 큐와 ROS QoS는 모두 최신 1프레임만 유지하므로 오래된 프레임이 쌓이지 않습니다.
- 우선 `400p`, `60 FPS`, `preview.fps: 5~10`으로 측정합니다.
- 처리 시간이 크면 `scan.pixel_stride`를 2로 올리거나 프리뷰를 끕니다.
- 더 안정적인 깊이가 필요할 때만 `high_accuracy`, subpixel 또는 큰 median filter를 켭니다.
- 터미널의 `delay`는 카메라 프레임 타임스탬프부터 LaserScan/프리뷰 처리 완료까지의 시간입니다.
