# depth_lidar

OAK 계열 스테레오 Depth 카메라를 직접 열어, 선택한 영상 ROI를 2D 라이다 형태의
`sensor_msgs/LaserScan`으로 변환하는 독립 ROS 2 테스트 패키지입니다. 연속 스캔 bin을
경량 군집화해 원형 장애물로 표시하며, CAM_A의 NV12도 변환이나 프리뷰 없이 동시에
호스트로 전송해 향후 BEV 파이프라인의 USB 부하를 재현합니다. 다른 주행 노드나 카메라
드라이버를 사용하지 않습니다.

## 출력

- `~/scan` (`/depth_lidar/scan`): x축 전방, y축 좌측인 `LaserScan`
- `~/preview` (`/depth_lidar/preview`): 검정 배경의 BEV 좌표계에 청록색 측정점과
  주황색 원형 장애물을 표시한 진단 영상
- 프리뷰/터미널: Depth 호스트 수신 FPS, NV12 호스트 수신 FPS, Depth 투영부터 군집과
  객체 위치 산출까지의 평균 연산시간 및 그 역수인 처리 가능 FPS
- 터미널: 참고용 카메라 촬영 시각부터 출력까지의 delay. 수신/연산 FPS에는 포함하지 않음

프리뷰 창이 필요하면 `preview.gui:=true`를 사용합니다. 기본값은 화면이 없는 Jetson에서도
동작하도록 이미지 토픽만 발행합니다.

NV12는 CAM_A에서 별도 최신 1프레임/non-blocking 큐와 전용 수신 스레드로 호스트까지
가져옵니다. 색 변환, ROS 발행, 파일 저장과 화면 출력은 하지 않습니다. 따라서 표시되는
`NV12 RX`는 Depth 처리 루프의 속도에 묶이지 않은 실제 호스트 수신률입니다.

## 빌드 및 실행

DepthAI C++ 3.x 라이브러리, OpenCV 및 ROS 2가 설치되어 있어야 합니다. OAK 장치는 이 노드가
직접 점유하므로 같은 장치를 쓰는 다른 카메라 노드를 먼저 종료합니다.

```bash
cd <workspace>
colcon build --packages-select depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py
```

다른 실험용 YAML을 실행 시점에 전달하려면 절대 경로를 사용합니다.

```bash
ros2 launch depth_lidar depth_lidar.launch.py \
  config_file:=/absolute/path/to/experiment.yaml
```

Launch를 거치지 않고 노드를 직접 실행할 때도 YAML을 전달할 수 있습니다.

```bash
ros2 run depth_lidar depth_lidar_node --ros-args \
  --params-file /absolute/path/to/experiment.yaml
```

YAML의 루트 키는 아래처럼 노드 이름인 `depth_lidar`여야 합니다.

```yaml
depth_lidar:
  ros__parameters:
    camera.fps: 60.0
    camera.resolution: "400p"
    roi.width_ratio: 1.0
    roi.height_ratio: 0.10
    roi.bottom_offset_ratio: 0.35
    range.max_m: 8.0
    range.offset_m: 0.0
    nv12.enabled: true
    nv12.fps: 60.0
    nv12.width: 1280
    nv12.height: 800
    bev.x_min_m: 0.0
    bev.x_max_m: 3.0
    bev.y_min_m: -0.6
    bev.y_max_m: 0.6
    bev.meter_per_pixel: 0.01
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
ros2 param set /depth_lidar range.offset_m -0.05
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
| `range.offset_m` | 거리 보정값. 음수는 가깝게, 양수는 멀게 보정 |
| `scan.bins` | ROI 횡방향 시야각을 나누는 LaserScan 각도 구간 개수 |
| `scan.pixel_stride` | ROI 픽셀 샘플 간격; 클수록 빠르지만 성긴 결과 |
| `scan.min_points_per_bin` | bin을 유효하게 만드는 최소 픽셀 수 |
| `cluster.min_bins` | 객체 군집으로 인정할 최소 유효 각도 bin 수 |
| `cluster.max_missing_bins` | 같은 군집 안에서 허용할 연속 누락 bin 수 |
| `cluster.base_neighbor_distance_m` | 인접 점 연결의 기본 거리 허용값 |
| `cluster.angular_neighbor_scale` | 거리에 따라 커지는 ray 간격의 보정 배율 |
| `cluster.radius_margin_m` | 추정 반폭에 더하는 안전 여유 |
| `cluster.min_radius_m`, `cluster.max_radius_m` | 원형 장애물 반지름 제한 |
| `nv12.enabled`, `nv12.fps` | CAM_A NV12 동시 호스트 전송과 요청 FPS |
| `nv12.width`, `nv12.height` | NV12 전송 해상도. 1280x800 이하의 짝수 크기 |
| `preview.enabled`, `preview.gui` | 프리뷰 토픽/GUI 사용 여부 |
| `preview.fps`, `preview.scale` | 프리뷰 갱신률과 BEV 정수 확대 배율 |
| `preview.size_px` | 이전 YAML 호환용. 현재 렌더러에서는 사용하지 않음 |
| `bev.*_m`, `bev.meter_per_pixel` | 프리뷰의 실제 BEV 범위와 해상도 |
| `sensor.x_m`, `sensor.y_m`, `sensor.yaw_deg` | 카메라 원점에서 앞차축 BEV 좌표로 가는 2D 장착 자세 |

### 군집과 원형 장애물

군집화는 유효 `LaserScan` bin을 각도 순서대로 한 번만 순회합니다. 인접 점의 실제 거리와
누락 bin 수가 설정 범위 이내면 같은 군집으로 취급하므로 DBSCAN이나 전체 점 쌍 비교가
필요 없습니다. 반지름은 원시 Depth 픽셀 개수가 아니라 군집의 각도 폭과 대표 거리로
계산합니다.

```text
physical width = 2 * representative range * sin(angular width / 2)
radius = clamp(physical width / 2 + margin, minimum radius, maximum radius)
```

따라서 같은 폭의 물체가 멀어져 점유 bin 수가 줄어도 거리 항으로 보정됩니다. 서로 붙어
연속된 물체는 하나의 큰 원형 장애물로 합쳐질 수 있으며, 이는 러프한 occupancy 시험을
위한 의도된 동작입니다.

프리뷰는 `bev_processor`와 같은 방식으로 화면 위쪽을 전방 `+X`, 화면 왼쪽을 차량 좌측
`+Y`로 사용합니다. 현재는 실제 차선 BEV를 입력받거나 합성하지 않고 검정 배경만 그립니다.
`sensor.*` 변환은 장애물과 점의 위치에만 적용되며 기존 `/depth_lidar/scan`은 계속 카메라
기준 `frame_id`로 발행됩니다.

### 거리 offset

카메라 기울기나 설치 위치를 완전한 3D 자세 추정 대신 상수 거리로 근사 보정합니다.

```text
최종 LaserScan 거리 = 카메라 측정 거리 + range.offset_m
```

예를 들어 실제 1.00m 장애물을 카메라가 1.05m로 측정한다면 `-0.05`를 설정합니다.
반대로 0.97m로 측정한다면 `+0.03`을 설정합니다. 보정값은 다음 프레임부터
즉시 반영되며 카메라 파이프라인은 재시작하지 않습니다.

### scan.bins와 연산량

`scan.bins`는 ROI의 횡방향 시야각을 몇 개의 각도 구간으로 표현할지 결정합니다.
값이 크면 각도 분해능은 높아지지만 `LaserScan.ranges` 배열과 프리뷰 점이 많아집니다.

현재 구현은 `scan.bins`와 관계없이 ROI의 샘플 픽셀을 모두 순회합니다. 따라서 bin을
360에서 180으로 줄여도 메인 Depth 투영 연산량은 거의 같고, 배열 초기화·ROS 전송·
프리뷰 출력 비용만 조금 줄어듭니다. FPS를 크게 올리려면 다음 순서가 더 효과적입니다.

1. `scan.pixel_stride`를 1에서 2로 올립니다.
2. `roi.height_ratio`나 `roi.width_ratio`를 줄입니다.
3. `preview.fps` 또는 `preview.size_px`를 줄입니다.
4. 필요하면 `camera.resolution`을 낮추습니다.

권장 상한은 다음과 같습니다.

```text
scan.bins <= ceil(ROI 폭 / scan.pixel_stride)
```

## 저지연 설정 팁

- DepthAI 출력 큐와 ROS QoS는 모두 최신 1프레임만 유지하므로 오래된 프레임이 쌓이지 않습니다.
- 우선 `400p`, `60 FPS`, `preview.fps: 5~10`으로 측정합니다.
- 처리 시간이 크면 `scan.pixel_stride`를 2로 올리거나 프리뷰를 끕니다.
- 더 안정적인 깊이가 필요할 때만 `high_accuracy`, subpixel 또는 큰 median filter를 켭니다.
- 터미널의 `delay`는 카메라 프레임 타임스탬프부터 LaserScan/프리뷰 처리 완료까지의 시간입니다.

카메라가 실행 중일 때 터미널 출력은 다음 형식입니다.

```text
target FPS 60.0 | actual FPS 58.7 | achievement 97.8% | delay 18.20 ms | processing 1.42 ms | ...
```
