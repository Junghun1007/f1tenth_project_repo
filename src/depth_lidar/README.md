# depth_lidar

OAK 계열 스테레오 Depth 카메라를 직접 열어, 선택한 영상 ROI를 2D 라이다 형태의
`sensor_msgs/LaserScan`으로 변환하는 독립 ROS 2 테스트 패키지입니다. 연속 스캔 bin을
경량 군집화하며, 프리뷰는 카메라 기준 측정점을 흰색 레이더 격자에 표시합니다.
CAM_A의 NV12도 변환이나 프리뷰 없이 동시에
호스트로 전송해 향후 BEV 파이프라인의 USB 부하를 재현합니다. 다른 주행 노드나 카메라
드라이버를 사용하지 않습니다.

## 출력

- `~/scan` (`/depth_lidar/scan`): x축 전방, y축 좌측인 `LaserScan`
- `~/preview` (`/depth_lidar/preview`): 흰색 배경에 거리 반원·각도선과 파란색 측정점을
  표시한 카메라 기준 레이더 영상. BEV/앞차축 변환과 원형 장애물 표시는 적용하지 않음
- `~/stereo_preview`: 선택적으로 켜는 좌우 정렬 영상 + 초록색 ROI 테두리
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
    preview.size_px: 700
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

## 스테레오 영상에서 ROI 조정

`config/depth_lidar_roi_test.yaml`은 레이더와 좌우 스테레오 GUI를 모두 켠 테스트 설정입니다.

```bash
ros2 launch depth_lidar depth_lidar.launch.py config_file:="$(ros2 pkg prefix depth_lidar)/share/depth_lidar/config/depth_lidar_roi_test.yaml"
```

`stereo_preview.enabled=true`, `stereo_preview.gui=true`로 켜면 좌우 정렬 영상을 나란히
표시합니다. 오른쪽 `RIGHT / DEPTH ROI`의 초록색 테두리가 실제 scan ROI입니다.
깊이 정렬을 기존 CENTER에서 RECTIFIED_RIGHT로 변경해 오른쪽 영상과 좌표를 맞췄습니다.
왼쪽 테두리는 동일 좌표의 비교용 가이드이며, 시차 때문에 물체의 가로 위치는 다릅니다.
Depth가 축소되는 preset에서도 실제 depth ROI를 영상 해상도에 맞춰 표시합니다.
레이더와 스테레오 창은 각각 최신 데이터로 갱신되며, 좌우 영상끼리는 같은 sequence를 사용합니다.

창에 포커스를 둔 채 **C**를 누르면 카메라 화면을 켜고 끕니다. 다음 명령도 가능합니다.

```bash
ros2 param set /depth_lidar stereo_preview.enabled false
ros2 param set /depth_lidar stereo_preview.enabled true
ros2 param set /depth_lidar stereo_preview.gui false
ros2 param set /depth_lidar stereo_preview.gui true
```

`enabled` 전환은 좌우 USB 전송을 추가/제거하기 위해 카메라 파이프라인을 잠깐 재시작합니다.
`gui`는 창만 제어하므로 재시작 없이 토픽과 전송을 유지합니다. 기본 설정은 둘 다 false입니다.
`stereo_preview.fps`는 호스트 변환·발행·표시 속도이며, 전송 속도는 camera.fps입니다.
ROI 파라미터는 실행 중 변경 시 다음 갱신에 반영됩니다. 결정한 값은 YAML에 직접 저장합니다.

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
| `scan.range_selection` | 각도별 유효 거리 선택: nearest(기본), farthest |
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
| `stereo_preview.enabled`, `stereo_preview.gui`, `stereo_preview.fps` | 좌우 ROI 프리뷰 전송, 창, 갱신률 |
| `preview.fps`, `preview.size_px` | 프리뷰 갱신률과 가로 픽셀 수; 세로 크기는 자동 결정 |
| `preview.scale`, `bev.*`, `sensor.*` | 이전 YAML 호환용. 현재 레이더 표시에는 적용하지 않음 |

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

### 카메라 기준 레이더 프리뷰

흰색 배경의 아래 중앙 십자표가 카메라 원점입니다. 위쪽은 카메라 전방, 왼쪽은
카메라 좌측이며 양의 각도입니다. 거리 반원 5개는 `range.max_m`까지 같은 간격으로
그립니다. 30도 간격 각도선과 ROI 좌우 시야 경계선을 함께 표시합니다.

파란색 점은 `/depth_lidar/scan`과 동일한 각도별 대표 측정값입니다.
`scan.range_selection=nearest`는 최근접, `farthest`는 최원거리 유효 값을 선택합니다. 원본 Depth의 모든
픽셀을 표시하는 것은 아니며, `range.offset_m` 등 기존 scan 처리는 그대로 적용됩니다.
바닥 제거는 수행하지 않습니다. 측정이 없으면 `NO VALID RETURNS`를 표시합니다.

```text
pixel_x = camera_origin_x - range * sin(angle) * pixels_per_meter
pixel_y = camera_origin_y - range * cos(angle) * pixels_per_meter
```

`preview.size_px`는 가로 크기이며 기본 700px입니다. `preview.scale`, `bev.*`, `sensor.*`는
이전 YAML을 그대로 읽을 수 있도록 유지하지만 레이더 표시에 영향을 주지 않습니다.
거리·각도는 카메라 기준이며 앞차축 이동이나 BEV 좌표 변환, 장애물 반지름 확대는
프리뷰에 적용하지 않습니다. 군집 연산은 성능 측정을 위해 계속 수행합니다.

### 최근접 / 최원거리 비교

ROI 테스트 YAML은 `scan.range_selection: "farthest"`로 설정되어 있습니다.
기본 패키지 YAML과 파라미터 생략 시에는 기존 `nearest` 동작을 유지합니다.
레이더에 현재 선택 모드를 표시하며 다음 명령으로 재시작 없이 바꿀 수 있습니다.

```bash
ros2 param set /depth_lidar scan.range_selection farthest
ros2 param set /depth_lidar scan.range_selection nearest
```

거리 보정과 min/max 범위 필터를 통과한 픽셀 중에서 각도별 최솟값/최댓값을 고릅니다.
유효 픽셀이 없거나 `scan.min_points_per_bin`보다 적으면 측정 없음으로 처리합니다.
선택 모드는 레이더뿐 아니라 `/depth_lidar/scan`과 군집 계산에도 적용됩니다.
두 방식 모두 지면 제거가 아닙니다. 예를 들어 한 각도 구간에 0.5m 물체와 2m 바닥이
함께 잡히면 farthest는 2m 바닥을 선택합니다. 지면과 물체를 구분하려면 높이 또는
지면 모델을 이용한 별도 필터가 필요합니다.

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
