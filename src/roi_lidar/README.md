# roi_lidar: RGB BEV + 높이 필터를 통과한 XY 포인트

실제 RGB 카메라의 지면 BEV 위에 스테레오 깊이 포인트를 **위에서 수직으로 내려다본
XY 위치**로 표시한다. 객체를 묶거나 외곽선을 그리지 않고, 설정한 최대 높이 이하의
관측 포인트를 그대로 그린다. 차선 인식은 수행하지 않는다.

예를 들어 같은 방향에서 1m와 2m 표면이 각각 관측되었다면 두 포인트를 모두 표시한다.
기존 `/scan`처럼 각도별 최근접점 하나로 줄인 결과를 프리뷰에 쓰지 않는다.
`points.pixel_stride`에 따른 이미지 샘플링, ROI, 거리/깊이/시야각 제한은 적용한다.
장애물 뒤의 가려진 표면은 복원하지 않는다.

## 시작과 실행

ROS 2 Humble, 기존 DepthAI C++ 3.6+ SDK, OpenCV와 OAK 장치가 필요하다.
SDK 경로는 [공통 설정](../../DEPTHAI_SETUP.md)을 따른다. SDK 재설치는 수행하지 않는다.
같은 OAK를 여는 camera_driver, bev_processor, point_cloud, depth_lidar 등은 종료한다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-up-to roi_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roi_lidar roi_lidar.launch.py \
  config_file:="$(pwd)/src/roi_lidar/config/roi_lidar.yaml"
```

평지에서 정지한 채 시작한다. `oak_startup`으로 지면과 카메라 높이·roll·pitch를
한 번 측정한다. 이 지면은 이후 고정이며 실시간 자세 보정이나 경사도 추정을 하지 않는다.
기존 point_cloud의 보정/깊이 입력 라이브러리를 공유하되 해당 노드나 RViz는 시작하지 않는다.
기본 좌표 원점은 앞차축 지면이며 +X는 전방, +Y는 차량 좌측이다.

기본 표시 영역은 폭 1.5m, 전방 4m로 미터당 픽셀 비율을 동일하게 유지한다.
최대 검출 거리는 별도의 `range.max_m`이며 화면 밖 포인트도 토픽에는 남는다.

## 화면과 조작

- **RGB BEV**: 실제 RGB 지면 투영 영상 위에 XY 포인트와 바닥 분류를 표시한다.
- **select depth ROI**: rectified-right 흑백 영상에서 사각형을 드래그한다.
  `preview.camera_image=false`이면 깊이 컬러 영상으로 바뀐다.
- **live controls**: 프리뷰를 유지하면서 높이, 지면 제거, 샘플링, 바닥 분류와 표시를 조절한다.
- **depth settings (A apply)**: 카메라 FPS, confidence, 좌우 일치 검사, subpixel,
  IR dot/flood를 조절한 뒤 **A**를 눌러 한 번에 적용한다. 이때 스트리밍은 재시작한다.

R은 전체 이미지 ROI로 복원하고, Q/Esc는 종료한다. GUI는 ROS 메인 스레드에서 처리한다.

라이브 슬라이더 변경은 카메라를 재시작하지 않는다. 슬라이더 위치에 이름의 `+숫자`를
더하면 표시 단위의 값이다. 예: `Max obstacle height (cm, +1)`에서 위치 49는 50cm다.
컨트롤 창 아래에는 실제 적용된 최대 Z, 지면 기준, 상한과 셀 크기를 m 단위로 표시한다.
유효하지 않은 조합은 적용하지 않고 원래 값으로 돌린 뒤 창과 로그에 이유를 표시한다.

지면 기준은 **base <= cap < max height**를 만족해야 한다. 최대 높이를 낮출 때는
필요하면 지면 제거 상한과 기본 두께부터 낮춘다. GUI의 최대 높이 범위는 1cm~3m이며
더 큰 값 등 GUI 범위 밖 설정은 YAML/`ros2 param set`으로 지정한다.
외부 ROS 파라미터 변경도 GUI에 반영한다.

## 포인트 높이 필터

각 depth 표본을 보정된 차량 좌표 `(X,Y,Z)`로 변환한다. Z는 시작할 때 측정한 지면
기준 높이(m)다. 다음 조건의 실제 표본을 `(X,Y,0)`에 표시한다.

```text
min(ground.max_distance_m,
    ground.distance_m + ground.distance_per_meter × hypot(X,Y)) < Z <= height.max_m
```

`ground.enabled=false`면 포인트의 지면 제거 하한을 해제한다. 최대 높이 제한은 유지한다.
픽셀 하나당 깊이는 하나뿐이므로 실제 LiDAR처럼 가려진 뒷면까지 반환되는 것은 아니다.
화면의 빨간 포인트는 필터를 통과한 원시 관측이며 셀의 최소 표본 수와 무관하다.

## 바닥·장애물·미관측 분류

현재 프레임의 실제 표본을 기본 5cm 셀에 집계하며 초록/빨강/회색으로 표시한다.
이 색상은 지면 관측 근거의 구분이며 경로 계획이나 차량의 최종 통과 판정이 아니다.

| 셀 | 조건 |
|---|---|
| 초록: 관측 바닥 | `-terrain.below_ground_m <= Z <= 거리별 지면 제거 두께`인 표본이 최소 수 이상이고 위험 표본은 없음 |
| 빨강: 위험 관측 | 지면 두께보다 높은 표본 또는 지면 아래 허용값보다 낮은 표본이 최소 수 이상 |
| 회색: 미관측/불확실 | 유효 깊이 없음, ROI 밖, 근거 부족, 또는 최소 수에 못 미치는 위험 표본이 있음 |

최대 높이를 넘는 표본은 제외한다. 같은 셀에 바닥과 충분한 위험 근거가 함께 있으면
위험이 우선한다. 위험 표본이 하나만 있어도 초록색으로 판정하지 않는다.
보이지 않는 공간, 광선 앞쪽과 장애물 뒤쪽을 초록색으로 채우지 않는다.
단, 실제 바닥이 별도로 관측된 셀은 차량과의 연결성 여부와 관계없이 초록색이 될 수 있다.
프레임 간 누적, 메모리, 셀 보간, 팽창, 군집 연결은 수행하지 않는다.
`ground.enabled`는 포인트 하한에만 적용되며 바닥 셀 분류 기준은 유지한다.

차량 폭 **0.30m**는 원점의 폭 눈금에만 반영한다. 차체 길이, 조향 궤적, 장애물 여유 간격,
연결된 통로 폭과 바닥 경사는 아직 평가하지 않는다. 녹색을 차량 전체가 통과 가능하다는
뜻으로 사용하지 않는다. 지면 아래 표본은 실제 함몰과 깊이 오류를 구별하지 않는다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `height.max_m` | 1.0 | 표시/발행/셀 집계에 사용할 지면 기준 최대 높이 |
| `ground.distance_m` | 0.05 | 지면 제거 기본 두께 |
| `ground.distance_per_meter` | 0.01 | 수평거리 1m당 두께 증가 |
| `ground.max_distance_m` | 0.12 | 지면 제거 두께 상한 |
| `terrain.below_ground_m` | 0.05 | 지면 아래 허용 두께 |
| `terrain.cell_m` | 0.05 | 셀 크기, 0.02~0.25m |
| `terrain.min_ground_samples` | 2 | 셀을 바닥으로 판정할 최소 표본 수 |
| `terrain.min_obstacle_samples` | 2 | 셀을 위험으로 판정할 최소 표본 수 |
| `terrain.vehicle_width_m` | 0.30 | 폭 표시 기준 |
| `points.pixel_stride` | 2 | ROI 이미지 표본 간격 |
| `preview.show_terrain` | true | 바닥 분류 색상 표시 |
| `preview.show_projected_points` | true | XY 포인트 표시 |
| `preview.terrain_alpha` | 0.40 | 색상 불투명도, 0이면 RGB 원본 유지 |
| `preview.point_radius` | 1 | XY 포인트 반경(px) |

RGB 보간은 `preview.interpolation=nearest/linear/cubic/lanczos4`이며 장애물 좌표에는
영향을 주지 않는다. 샘플링을 줄이면 바닥 셀의 근거도 줄어 회색 영역이 늘 수 있다.
셀 크기·최소 표본 수를 함께 조절한다. 큰 지면 두께는 낮은 장애물을 바닥으로 처리할 수 있다.

## 토픽

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/roi_lidar/obstacle_points` | PointCloud2 | 모든 높이 필터 통과 XY 포인트. x/y/z는 float32(m), z=0, `height`에 원래 지면 기준 Z(m). 화면 밖도 포함 |
| `/roi_lidar/terrain` | OccupancyGrid | -1=미관측/불확실, 0=관측 바닥, 100=위험 관측. X방향 열, Y방향 행. 원점=(0,-표시폭/2), resolution=셀 크기 |
| `/roi_lidar/scan` | LaserScan | 기존 각도별 최근접 반환점. 프리뷰와 새 포인트 토픽의 입력으로 쓰지 않음 |
| `/roi_lidar/bev_preview` | Image bgr8 | RGB BEV + 현재 XY 포인트/바닥 색상 |
| `/roi_lidar/roi_preview` | Image bgr8 | ROI 지정용 영상 |
| `/roi_lidar/status` | String | 측정/입력중단/오류, transient local |

`/obstacles` PoseArray와 `cluster.*`는 이전 기능의 선택적 호환 출력으로 남겼으며
기본 `cluster.enabled=false`다. 켜더라도 새 프리뷰의 XY 포인트는 묶지 않는다.
`preview.show_raw_points`는 예전 프리뷰 설정이며 새 XY 화면에서는
`preview.show_projected_points`를 사용한다.

RGB/depth가 모두 신선하지만 시각 차이가 `preview.max_sync_sec`를 넘으면 프리뷰의
깊이 overlay를 숨긴다. RGB가 없는 동안에는 신선한 깊이 결과를 단색 배경에 표시한다.
새 데이터 토픽은 RGB 시각과 무관하게 깊이 촬영 시각으로 발행한다.
깊이 입력 중단/오류 시 포인트는 비우고 셀은 미관측으로 초기화한다.
GUI가 없는 환경은 `gui:=false`로 영상/데이터 토픽을 사용한다.

## 조절값 저장

슬라이더·ROI 변경은 원본 YAML에 자동 저장하지 않는다.

```bash
ros2 param dump /roi_lidar > roi_lidar_test.yaml
ros2 launch roi_lidar roi_lidar.launch.py config_file:="$(pwd)/roi_lidar_test.yaml"
```

## 검증

```bash
colcon test --packages-select roi_lidar --event-handlers console_direct+
colcon test-result --verbose
```

ROI/바닥 제거/거리/좌표축/기울기, 현재 프레임 표본의 보존, 미관측 셀, 바닥/위험 충돌,
함몰 근거, 프레임 초기화, 높이 상한과 XY 투영 및 색상을 테스트한다.
실제 장치의 FPS·오검출·시작 지면 측정과 ROS 전체 빌드/GUI 동작은 차량 PC에서 확인한다.

컨트롤의 실시간 높이 적용, 단위 변환, A 키 전 카메라 설정 보류와 외부 파라미터 동기화는
`test/controls_test.cpp`로 검증한다. 실제 데스크톱 창을 열므로 기본 테스트에서는 제외한다.
디스플레이가 있는 환경에서 `-DROI_LIDAR_GUI_TESTS=ON`으로 빌드하면 함께 실행할 수 있다.
