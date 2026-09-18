# point_cloud

OAK의 **한 depth 프레임을 원본 해상도의 XYZ 점군으로 표시**하는 ROS 2 패키지다.
해상도와 IR 조명을 바꾸면서 바닥·벽·장애물의 깊이 품질을 비교한다.
원본 점군, 바닥 제거 점군, **BEV 차량 좌표로 변환하고 표시 영역을 제한한 3D 점군**을 발행한다.
시작 시 `oak_startup`으로 BEV와 같은 방식의 카메라 높이·roll·pitch 측정을 수행한다.
BEV 점군을 XY 클러스터로 묶고 **면적·두께가 있는 덩어리 형태까지 통과한 포인트만**
기본 RViz 프리뷰에 표시한다.
클러스터링 전에 최근 프레임의 3D 위치 반복성을 검사한다. 물체 ID 추적이나 과거 점의 재표시는 하지 않는다.
일반 점군 생성의 기본 요청 FPS는 50이며 실제 속도는 로그로 확인한다.

여기서 원본은 **StereoDepth가 계산한 depth의 유효 픽셀 전체**를 뜻한다. 센서 RAW 영상이나
필터 이전 disparity가 아니다. 스테레오 매칭·신뢰도·좌우 일치 검사에 의해 무효화된 점은
복구하지 않는다. 기본값은 추가 median/공간/점 잡음/홀 채우기 필터를 끄며, 설정으로 켤 수 있다.

## 빌드와 실행

기존 프로젝트와 같은 ROS 2 C++ 환경 및 **DepthAI C++ 3.6+**가 필요하다.
CAM_B/CAM_C 스테레오 카메라를 사용한다. 기본 dot 강도 0.5는 이를 지원하는 OAK Pro용이다.
같은 OAK를 점유하는 `bev_processor`, `camera_driver`, `depth_lidar`, `ir_camera_driver` 등은 먼저 종료한다.
이 실행은 독립 RViz 확인용이다. BEV 노드와 동시에 카메라를 열지 않는다.
`oak_startup` 의존성이 추가되어 `--packages-up-to`로 함께 빌드한다. BEV CUDA 노드 빌드는 필요 없다.

```bash
cd /path/to/f1tenth_project_repo
colcon build --packages-up-to point_cloud --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch point_cloud point_cloud.launch.py
```

RViz가 함께 열린다. 먼저 차량을 평평한 바닥에 정지시키고 중앙 측정 영역에서 장애물을 치운다.
BEV와 같은 시작 측정이 완료되면 **앞차축 지면 중심 기준의 장애물 클러스터 포인트만** 표시한다.
녹색 테두리는 지면 Z=0에서의 BEV 범위다. 점의 높이는 그대로 유지하며 회전/확대할 수 있다.
클러스터 프리뷰는 묶음별 색상을 사용한다. 기존 원본/BEV 디버그 표시는 전방 거리 색상을 유지한다.
`Decay Time: 0`은 최신 점군만 표시한다. Axes는 앞차축 지면 중심이다.
장치 연결이나 설정 적용 오류는 터미널과 `/point_cloud/status`에 표시한다.

```bash
# 고해상도 비교
ros2 launch point_cloud point_cloud.launch.py resolution:=800p fps:=30.0 dot:=0.5

# Dot 프로젝터를 지원하지 않는 OAK 또는 IR OFF 비교
ros2 launch point_cloud point_cloud.launch.py resolution:=400p dot:=0.0

# 사용자 설정 파일 / RViz 없이 발행
ros2 launch point_cloud point_cloud.launch.py config_file:=/absolute/path/point_cloud.yaml rviz:=false
```

`resolution`, `fps`, `dot`, `flood`, `ground`, `bev` launch 인자를 생략하면 YAML 값이 적용된다.
설정 파일은 `config/point_cloud.yaml`이다. 소스 YAML을 수정했다면 다시 빌드하거나
`config_file`에 수정한 파일의 절대 경로를 전달한다.

## BEV와 같은 영역의 3D 점군

기본 범위는 전방 X=0~3m, 좌우 Y=-0.6~0.6m이다. 깊이의 광축 Z를 3m에서 자르는 방식이 아니라
오른쪽 rectified 카메라 → CAM_A(RGB) → 앞차축 차량 좌표로 변환한 뒤 X/Y 범위를 검사한다.
범위는 `min <= 좌표 < max`이며 높이 Z를 평면으로 누르거나 별도 높이 범위로 자르지 않는다.
원본 및 바닥 제거 토픽은 기존 광학 좌표를 유지하고, `/point_cloud/points_bev`만 차량 좌표를 사용한다.

기본 launch는 빌드할 때 `bev_processor/config/bev_config.yaml`에서 설치한
`config/bev_reference.yaml`을 읽는다. BEV의 범위, CAM_A 장착 X/Y/yaw와 시작 측정 설정을 가져오며
point_cloud YAML에 같은 항목을 명시하면 그 값이 우선한다. 수정한 BEV YAML을 바로 쓰려면:

```bash
ros2 launch point_cloud point_cloud.launch.py \
  bev_config_file:="$(pwd)/src/bev_processor/config/bev_config.yaml"

# 실행 중 영역 제한 OFF / ON: 좌표 변환과 바닥 제거 설정은 유지
ros2 param set /point_cloud bev.enabled false
ros2 param set /point_cloud bev.enabled true

# 필요하면 거리 범위 변경 (카메라 재시작 없음)
ros2 param set /point_cloud bev.x_max_m 2.0
```

RViz에서는 **Obstacle cluster points**와 **BEV ground footprint**만 켜져 있다.
클러스터 판정 전 BEV 점군은 **BEV region 3D points**를 켜서 비교할 수 있다.
범위 밖의 점이 보이면 기존 `Ground-filtered point cloud` 또는 `Original point cloud`가
동시에 켜져 있는지 확인한다. `bev.enabled=false`일 때도 테두리는 BEV 기준 영역으로 남는다.
위에서 보려면 RViz Views의 Type을 `TopDownOrtho`로 바꾸고 Target Frame을 `front_axle_bev`로 둔다.

CAM_A 높이·자세는 `bev_processor`와 같은 `oak_startup` 구현으로 **이 실행에서 다시 측정**한다.
프레임 메타데이터의 실제 rectification 변환과 EEPROM의 센서 사이 회전·이동을 합성하여
오른쪽 카메라를 RGB 카메라 원점과 혼동하지 않는다. 측정/보정이 실패하면 BEV 점을 발행하지 않고
오류를 표시한다. 이미 측정한 자세는 카메라 설정 변경/재연결 때 유지하며, 차량 장착 자세나
노면 기준이 바뀌면 노드를 재시작하여 다시 측정한다.

현재 기능은 **BEV의 사각형 지면 범위에 들어오는 관측된 3D 점**을 보여준다.
RGB BEV의 유효 픽셀 마스크까지 적용하지는 않으며, 깊이 카메라가 관측하지 못한 곳의 점을 만들지 않는다.
BEV 실행과 별도 측정하므로 측정 오차가 있을 수 있고, 주행 중 IMU 흔들림 보정/프레임 동기화는
아직 이 점군에 적용하지 않는다. 우선 정지 상태에서 영역과 장애물 위치를 확인한다.

## 깜빡이는 점의 반복성 검사

기본 ON인 `temporal.*` 필터는 클러스터 전체가 아니라 **3D 위치별 관측 기록**을 검사한다.
따라서 콘과 연결된 선 모양 점도 반복성이 낮으면 클러스터링 전에 제외할 수 있다.
XY만 비교하지 않고 Z도 비교하므로 같은 바닥 위치의 서로 다른 높이를 혼동하지 않는다.

기본값은 현재 프레임을 포함한 **최근 5프레임 중 4프레임에서 4cm 이내로 관측**되어야 통과한다.
4cm 3D 격자에 점을 집계하고 프레임마다 격자 중심을 비교하므로 근사적인 위치 검사다.
한 프레임에 점이 많이 찍혀도 관측 횟수는 1번이다. 기록은 통과 여부와 관계없이 수집한다.
바닥 필터 및 BEV 범위 제한 후 클러스터 후보 높이에 들어오는 점만 기록한다.

출력은 **이번 프레임에서 실제 관측된 원래 점**이며 좌표를 평균내거나 사라진 점을 재생하지 않는다.
그래서 갑자기 생기는 깜빡임을 줄이지만, 실제 물체가 관측되지 않은 프레임까지 메워주지는 않는다.
원본/바닥 필터/BEV 디버그 토픽에는 시간 필터를 적용하지 않고, 클러스터 프리뷰에만 반영한다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `temporal.enabled` | true | 반복성 검사 ON/OFF. OFF면 기존 프레임별 클러스터 판정. |
| `temporal.voxel_size_m` | 0.04 | 기록을 집계할 3D 격자 크기. 작으면 세밀하지만 비용 증가. |
| `temporal.match_distance_m` | 0.04 | 이전 프레임 격자 중심과 현재 중심의 최대 XYZ 거리. 크면 흔들림 허용 증가, 주변 잡음도 통과 가능. |
| `temporal.window_frames` | 5 | 현재를 포함해 검사할 최근 fresh 프레임 수. |
| `temporal.min_hits` | 4 | 그중 반복 관측되어야 하는 프레임 수. 올리면 엄격해지고 검출 지연 증가. |
| `temporal.max_age_sec` | 0.25 | 기록의 최대 나이. 처리 FPS가 낮으면 window보다 기록이 적을 수 있음. |

```bash
# 켜기 / 비교용 끄기: 카메라 재시작 없음
ros2 param set /point_cloud temporal.enabled true
ros2 param set /point_cloud temporal.enabled false

# 최근 5프레임에 모두 나타난 위치만 통과시키기
ros2 param set /point_cloud temporal.min_hits 5

# 진짜 물체 점도 너무 많이 사라지면 완화
ros2 param set /point_cloud temporal.min_hits 3
```

조건 변경 후에는 기록을 비우고 다시 수집한다. 카메라 재연결/프레임 공급 중단/설정 변경 시에도
과거 조건의 기록을 재사용하지 않는다. 기본값의 최초 통과는 실제 50FPS일 때 최소 약 60ms 후이며,
검출 상태·실제 FPS·처리 지연에 따라 더 늦어진다. `min_hits <= window_frames`여야 하고
최대 기록 나이 안에 충분한 프레임이 들어와야 한다. 연결 허용 거리는 voxel_size의 2배 이하다.

**현재는 차량이 정지한 상태의 BEV 좌표를 비교한다.** 차량이나 물체가 움직이면 위치가 바뀌어서
진짜 장애물도 늦게 보이거나 제외될 수 있다. 주행 중 사용하려면 프레임 사이 차량 움직임을 보정한
공통 좌표 비교가 필요하다. 우선 정지 상태에서 콘과 선 모양 잡음이 분리되는지 확인한다.
계속 같은 위치에 생기는 잘못된 깊이 점은 이 검사로도 통과할 수 있다.
이 기능은 물체 추적이 아니므로 클러스터 색상/ID는 여전히 프레임마다 바뀔 수 있다.

## BEV 덩어리 형태 검사

`blob.*`는 높이·시간·클러스터 조건을 통과한 후보를 마지막으로 검사한다. 기본 ON이다.
포인트가 완벽한 폐곡선을 이루어야 하는 조건은 아니다. BEV에서 **실제로 면적과 두께가 있는
덩어리인지** 검사하며, 콘에 붙은 얇은 선 때문에 콘 전체를 버리지 않도록 선 부분을 먼저 잘라낸다.

1. 각 클러스터를 별도의 **1cm 점유 격자**에 표시한다.
2. **Closing**으로 작은 틈을 메우고, **Opening**으로 가느다란 가지·연결부를 제거한다.
   기본은 두 연산 모두 반경 1칸, 즉 3×3칸 커널이다.
3. 끊어진 영역들을 별도 덩어리로 다시 나눈다. 점유 면적, 회전된 외접 사각형의 짧은 변,
   내부 점유율, 긴 변/짧은 변 비율을 검사한다. 대각선도 두께와 가늘기를 실제 방향에 맞춰 검사한다.
4. 분리된 덩어리마다 최소 점 개수와 높은 점 개수/비율을 다시 확인한다.
   원래 묶음에 높은 점이 있었다는 이유로 떨어져 나온 낮은 덩어리가 통과하지 않는다.
5. **최종 영역 안에 남은 실제 3D 포인트만** 기존 `/point_cloud/points_clusters` 토픽으로 표시한다.
   틈을 메운 가상 픽셀은 포인트로 생성하지 않으며 면적 증거에도 포함하지 않는다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `blob.enabled` | true | 형태 검사 ON/OFF. OFF는 이전 높이 중심 클러스터 결과. |
| `blob.cell_size_m` | 0.01 | 형태 검사 전용 BEV 격자 크기. cluster 격자와 독립. |
| `blob.closing_radius_cells` | 1 | 작은 틈을 메우는 커널 반경. 1=3×3, 2=5×5, 0=끄기. |
| `blob.opening_radius_cells` | 1 | 얇은 가지·연결부를 제거하는 커널 반경. 크게 하면 실제 물체도 더 깎임. |
| `blob.min_area_m2` | 0.0009 | 살아남은 원래 점유 면적 최소값. **9cm²**, 점 개수가 아님. |
| `blob.min_thickness_m` | 0.03 | 회전 외접 사각형의 짧은 변 최소값. **3cm**. |
| `blob.min_fill_ratio` | 0.35 | 원래 점유 면적 / 회전 외접 사각형 면적의 최소 비율. 빈 윤곽/성긴 무늬 제외. |
| `blob.max_aspect_ratio` | 6.0 | 긴 변/짧은 변의 상한. 너무 길쭉한 띠 제외. |

```bash
# 이전 방식과 비교
ros2 param set /point_cloud blob.enabled false
ros2 param set /point_cloud blob.enabled true

# 얇은 선을 더 강하게 제거 (5×5칸)
ros2 param set /point_cloud blob.opening_radius_cells 2

# 작은 콘까지 사라지면 기본 반경 유지 후 면적/두께 조건 완화
ros2 param set /point_cloud blob.opening_radius_cells 1
ros2 param set /point_cloud blob.min_area_m2 0.0004
ros2 param set /point_cloud blob.min_thickness_m 0.02
```

`blob.*`는 실행 중 변경 가능하며 카메라 재시작 없이 적용한다. 설정 변경으로 시간 기록은 초기화된다.
커널 반경을 칸 단위로 지정하므로 cell_size를 바꾸면 제거하는 실제 폭도 함께 바뀐다.
성긴 물체가 Opening에서 이미 사라졌다면 면적 문턱값만 낮춰도 복구되지 않는다.
그때는 opening 반경을 줄이거나 0으로 비교한다. Closing을 크게 하면 실제로 떨어진 물체도 붙을 수 있다.
얇은 막대·벽이나 깊이 카메라에 얇은 표면으로만 보이는 물체도 제외될 수 있다.
이 방식은 콘과 같은 덩어리 모양을 확인하기 위한 것이며 모든 종류의 장애물을 보존하지는 않는다.

기본 BEV 범위에서는 작은 격자만 필요하다. 범위 제한을 끈 경우에도 후보당 100만 칸을 넘으면
해당 후보를 제외하고 경고하여 큰 외곽/이상점 때문에 메모리를 과도하게 할당하지 않는다.

## 장애물 클러스터 프리뷰

`/point_cloud/points_bev` → 높이 후보 선택/3D 위치 반복성 검사 → 높은 점만 XY 연결 → 제한된 밑부분 포함 → 클러스터 판정 → BEV 덩어리 형태 검사 →
`/point_cloud/points_clusters` 순서다. RViz 기본 체크는 **Obstacle cluster points** 하나이며,
검출 결과가 없으면 빈 점군을 발행해 직전 장애물이 남지 않게 한다. 영역 테두리와 원점 축은 유지한다.
BEV 이미지에 합성하는 단계는 아직 없으며, 이번 프리뷰는 동일 차량 좌표에서의 3D 포인트 표시다.

- 먼저 시작 측정 지면 기준 Z=3cm 미만/2m 초과 점을 후보에서 제외한다.
- **높이 6cm 이상인 점만** XY 2cm 격자에 집계하여 연결 판단에 사용한다. 높은 점의 격자 중심끼리
  8cm 이내면 연결한다. 낮은 점은 중심 위치 계산이나 연결 경로에 들어가지 않는다.
- 높은 묶음 자체에 최소 5점과 최소 수평 폭 3cm가 있어야 장애물 중심부 후보가 된다.
  낮은 바닥 점으로 이 높이 증거의 크기나 개수를 부풀릴 수 없다.
- 높이 3~6cm 사이의 점은 중심부의 높은 격자 중심에서 XY 거리 **2cm 이내인 경우에만 직접 포함**한다.
  추가한 낮은 점을 발판으로 또 다른 낮은 점을 연결하지 않는다. 두 중심부의 밑부분이 겹쳐도
  가장 가까운 중심부 하나에만 배정하며, 이 때문에 중심부들이 합쳐지지는 않는다.
- 마지막으로 중심부+허용 밑부분의 원본 점 20개 이상, 높은 점 비율 30% 이상을 확인한다.
  통과한 묶음의 **원래 3D 점**만 표시한다. 격자 중심으로 대체하지 않고 높이도 유지한다.

이전의 전체 후보 연결 방식에서는 낮은 점들이 넓게 이어지면 콘과 한 묶음이 될 수 있었다.
현재는 **높은 중심부 → 제한된 밑부분** 순서로만 구성하므로, 반복 관측되는 낮은 바닥 띠에도
시간 필터와 독립적으로 작동한다. 다만 중심부 주변의 허용 반경 안에는 낮은 잡음이 일부 남을 수 있다.
`cluster.base_radius_m=0`으로 설정하면 밑부분 추가를 완전히 끄고 높은 중심부만 표시한다.

모든 `cluster.*` 파라미터는 실행 중 변경 가능하며 카메라를 재시작하지 않는다.
색상과 `cluster_id`는 한 프레임 안에서 묶음을 구분하는 용도이며 프레임 간 고정 ID가 아니다.
위치 반복성 필터와 별개로 물체 ID 추적/사라진 검출 유지/이동 보정은 적용하지 않는다. 높이가 실제 장애물과 비슷하게 잘못 측정된
바닥 잔여점은 이 조건만으로도 통과할 수 있으므로 아래 설정을 실제 장면과 비교해 조정한다.

| 파라미터 | 기본값 | 의미 / 조절 방향 |
|---|---:|---|
| `cluster.cell_size_m` | 0.02 | XY 격자 해상도. 연결 중심은 높은 점만으로 계산. 작으면 세밀하지만 비용 증가. |
| `cluster.tolerance_m` | 0.08 | 높은 중심부 사이의 연결 거리. 낮은 바닥 점은 연결에 참여하지 않음. |
| `cluster.min_points` | 20 | 묶음의 최소 원본 후보 점 개수. 올리면 작은 잡음 제외, 먼/작은 물체도 탈락 가능. |
| `cluster.min_height_m` | 0.03 | 묶기 전 최소 지면 위 높이. 올리면 낮은 바닥 연결점과 낮은 장애물 점 모두 제외. |
| `cluster.max_height_m` | 2.0 | 묶기 전 최대 지면 위 높이. |
| `cluster.support_height_m` | 0.06 | 중심부를 만들 수 있는 최소 높이. 반드시 min_height보다 커야 함. |
| `cluster.min_support_points` | 5 | 위 높이 이상의 점 최소 개수. 높이 잡음 하나로 통과하지 않게 함. |
| `cluster.min_support_ratio` | 0.30 | 위 높이 이상의 점 / 묶음 전체 후보 점 비율의 하한. |
| `cluster.min_extent_m` | 0.03 | 높은 중심부만으로 계산한 max(X 폭, Y 폭) 최소값. |
| `cluster.base_radius_m` | 0.02 | 높은 격자 중심 주변에서 낮은 점을 추가할 XY 거리. 0이면 높은 중심부만 표시. |

```bash
# 밑부분을 제외하고 높은 중심부만 비교
ros2 param set /point_cloud cluster.base_radius_m 0.0

# 기본값: 중심부 주변 2cm까지 낮은 점 허용
ros2 param set /point_cloud cluster.base_radius_m 0.02

# 물체가 여러 조각으로 갈라질 때 연결 거리 조절
ros2 param set /point_cloud cluster.tolerance_m 0.10

# 작은 잡음 묶음을 더 엄격히 제외
ros2 param set /point_cloud cluster.min_points 30
ros2 param set /point_cloud cluster.min_support_ratio 0.5

# 낮은 물체도 후보로 볼 때 (높이 증거 기준도 같이 검토)
ros2 param set /point_cloud cluster.min_height_m 0.02
ros2 param set /point_cloud cluster.support_height_m 0.04
```

단위는 미터다. cell_size는 0.005~0.2m, 연결 거리는 cell_size 이상이며 0.5m 및
cell_size의 10배 이하로 제한한다. `min_height < support_height <= max_height`여야 한다.
예전에 사용한 `min_height=0.03`, `support_height=0.03` 조합은 낮은 점과 높은 중심부를 구분하지
못하므로 이제 거부한다. support_height는 우선 기본값 0.06으로 비교한다.
base_radius는 0~0.2m이고 cell_size의 10배 이하다.
점 개수는 격자 수가 아닌 **후보 원본 점 수**라 해상도나 `points.pixel_stride`를 바꾸면 다시 조정한다.
격자 집계 때문에 연결은 근사적이며 좁은 틈을 구분하려면 cell_size와 tolerance를 함께 줄인다.

## 바닥 제거 켜기 / 끄기

```bash
# 기본 ON. 처음부터 원본으로 보기
ros2 launch point_cloud point_cloud.launch.py ground:=false

# 실행 중 ON / OFF (카메라 재시작 없이 다음 프레임부터 적용)
ros2 param set /point_cloud ground.enabled true
ros2 param set /point_cloud ground.enabled false

# 바닥 두께가 남을 때 제거 허용 거리 조절 (미터)
ros2 param set /point_cloud ground.distance_m 0.04
```

RViz 왼쪽 Displays에서 **Obstacle cluster points**가 기본 체크되어 있다.
**Original point cloud (includes ground)**를 체크하고 클러스터 점군을 해제하면 전체 원본을 비교할 수 있다.
두 항목을 동시에 체크하면 원본의 바닥도 겹쳐 보인다. `ground.enabled=false`면
`points_filtered`는 원본을 그대로 표시하고, `points_bev`는 바닥을 포함한 BEV 영역을 표시한다.
클러스터 프리뷰는 별도의 높이·크기 조건을 계속 적용하므로 ground OFF만으로 전체 바닥이 표시되지는 않는다.
원본 `/point_cloud/points`는 항상 보존된다.

현재 프레임의 하단 절반에서 최대 2,000점을 샘플링하고 RANSAC으로 바닥 평면을 찾는다.
카메라 아래 방향과의 각도, 카메라와 평면의 수직 거리, 지지점 비율/분포로 후보를 제한하여
수직 벽을 제외한다. 기본 제거 범위는 광축 거리 3m 이내, 바닥 평면에서 ±3cm이다.
이 광축 범위 밖 점은 바닥 필터 단계에서는 유지한다. 이후 BEV 토픽에만 차량 좌표 변환과 X/Y 제한을 적용한다.
제거한 점은 XYZ 모두 NaN으로 바꾸어 organized cloud 구조를 유지한다.

바닥을 충분히 찾지 못하면 해당 프레임은 원본을 표시하고 5초 간격으로 경고한다.
기본 카메라 높이 후보 범위는 8~50cm, 기울기는 45도 이내다. 실제 장착 조건에 맞게
`ground.min_height_m`, `ground.max_height_m`, `ground.max_tilt_deg`를 조절한다.
바닥이 거의 안 보이거나 계단/경사 변화가 있는 장면에서는 검출이 실패하거나 표시가 흔들릴 수 있다.
높이 범위에 들어오는 넓은 다른 평면도 바닥으로 오인할 수 있으므로 원본과 비교해 범위를 좁힌다.
허용 거리를 크게 하면 낮은 장애물의 점도 제거되므로 필요한 만큼만 올린다.
이 기능은 시각 확인용이며 주행용 장애물 판정으로 검증한 것은 아니다.

## 실행 중 설정 변경

```bash
ros2 param set /point_cloud camera.resolution 800p
ros2 param set /point_cloud camera.fps 15.0
ros2 param set /point_cloud depth.ir_dot_projector_intensity 0.8
ros2 param set /point_cloud depth.ir_flood_light_intensity 0.2
ros2 param set /point_cloud depth.confidence_threshold 150
ros2 param set /point_cloud depth.subpixel true
ros2 param set /point_cloud depth.median_filter 3x3
```

위 카메라/깊이 설정 변경이 확정되면 **파이프라인을 닫고 같은 장치를 다시 열어 적용**한다.
이때 잠시 점군 출력이 멈추며 IR 강도도 매번 재적용한다.
`blob.*`, `temporal.*`, `cluster.*`, `ground.*` 또는 BEV 영역 설정만 변경하면 카메라를 재시작하지 않고 호스트 처리에 적용한다. 파라미터 서비스의 성공은 값 검증/저장 성공을 뜻한다.
실제 장치 적용 성공은 `/point_cloud/status`의 `STREAMING`과 새 프레임 로그로 확인한다.
지원하지 않는 조명/모드 조합은 오류로 표시하며 설정을 몰래 낮추지 않는다.
오류 상태에서도 파라미터를 수정할 수 있고, 2초 간격으로 연결을 재시도한다.
`device_id`, `frame_id`, `view_frame_id`, `bev.frame_id`, `bev.camera_*`,
`measurement_*`, `manual_camera_height_*`는 시작할 때만 설정한다.

## 해상도 선택

| 설정 | 요청 스테레오 입력 크기 | stride=1 점 슬롯/프레임 | XYZ 데이터/프레임 |
|---|---:|---:|---:|
| `400p` | 640×400 | 256,000 | 약 3.07 MB |
| `480p` | 640×480 | 307,200 | 약 3.69 MB |
| `720p` | 1280×720 | 921,600 | 약 11.06 MB |
| `800p` | 1280×800 | 1,024,000 | 약 12.29 MB |

요청 크기에 대한 수치이며, 무효 픽셀도 NaN 슬롯을 차지한다. 실제 크기와 카메라 내부
파라미터는 수신 프레임에서 읽어 로그로 표시한다. 카메라/DepthAI가 지원하는 모드만
동작한다. 비율이 다른 해상도는 크롭/FOV도 달라질 수 있어 단순 픽셀 수 비교와 구별한다.

800p·30 FPS의 XYZ만 약 369 MB/s이며 이는 **호스트 ROS 메시지 데이터량**이다.
원본/필터/BEV 세 토픽의 XYZ는 각각 같은 슬롯 수다. 클러스터 토픽은 통과한 점만
20 bytes/point(XYZ, RGB, cluster_id)로 압축해서 발행한다.
영역 밖 점도 NaN 슬롯을 유지하므로 영역 제한 자체가 메시지 바이트 수를 줄이지는 않는다.
USB에서는 16비트 depth를 수신하고 호스트가 XYZ로 변환하므로 XYZ 수치가 USB 사용량은 아니다.
ROS 직렬화, RViz, depth 이미지 발행 비용이 더해진다. 로그의 FPS는 카메라 설정값이 아닌
처리·발행한 fresh frame의 실제 속도이며, 큐는 최신 1개를 유지하고 밀린 프레임은 버린다.

처음에는 동일한 장면/조명/스테레오 설정에서 400p와 800p를 비교한다.
작은 장애물의 점 밀도, 바닥의 두께/흔들림, 물체 경계, 유효 점 비율, FPS를 함께 본다.
로그의 `host`는 XYZ 복원·바닥 필터·BEV 변환/영역 제한·위치 반복성 검사·클러스터링·덩어리 형태 검사·메시지 생성/발행 호출 시간이며 카메라 연산과 RViz 렌더링은 제외한다.
`age`는 수신 시점의 프레임 나이다. 워밍업 구간을 지난 로그로 비교한다.

## 주요 옵션

| 파라미터 | 의미 |
|---|---|
| `depth.ir_dot_projector_intensity` | 0~1 정규화 강도, 0=OFF. mA 단위가 아니다. |
| `depth.ir_flood_light_intensity` | 별도 IR flood 조명 강도, 0~1. |
| `depth.mode` | `default`, `high_density`(FAST_DENSITY), `high_accuracy`(FAST_ACCURACY). 이후 명시적 옵션이 프리셋을 덮어쓴다. |
| `depth.confidence_threshold` | 0~255. 낮출수록 엄격하여 무효 픽셀이 늘어날 수 있다. |
| `depth.left_right_check` / `left_right_threshold` | 좌우 일치 검사와 오차 문턱값. |
| `depth.subpixel` / `subpixel_fractional_bits` | 세밀한 disparity. 3/4/5비트. 4/5비트는 median을 먼저 꺼야 한다. |
| `depth.extended_disparity` | 근거리용 disparity 범위 확대. 이 패키지에서는 subpixel과 동시 사용을 거부한다. |
| `depth.median_filter` | `off`, `3x3`, `5x5`, `7x7`. 장치별 지원 범위가 다르다. |
| `depth.spatial_filter` / `speckle_filter` | 공간 평활화 / 점 잡음 필터. 기본 OFF. |
| `depth.hole_filling` / `adaptive_median_filter` | 홀 채우기 / 적응형 median. 기본 OFF. |
| `points.pixel_stride` | 1=전체. 2 이상은 호스트 출력 샘플링이며 카메라 해상도/USB depth 크기는 바뀌지 않는다. |
| `points.min_depth_m` / `max_depth_m` | 선택적 광축 Z 범위. max=0이면 상한 없음. 기본 0/0으로 추가 거리 제한 없음. |
| `bev.enabled` | BEV XY 범위 제한 ON/OFF. 기본 true. OFF여도 차량 좌표 변환은 유지. |
| `bev.x_min_m` / `x_max_m` | 전방 범위. launch에서 BEV YAML을 읽으며 기본 0~3m. |
| `bev.y_min_m` / `y_max_m` | 좌우 범위. launch에서 BEV YAML을 읽으며 기본 -0.6~0.6m. |
| `ground.enabled` | 바닥 제거 ON/OFF. 기본 true, 원본 토픽에는 영향 없음. |
| `ground.distance_m` | 평면에서 제거할 거리. 기본 0.03m, 허용 (0, 0.10]. |
| `ground.max_depth_m` | 평면 추정 및 제거 범위의 광축 Z 상한. 기본 3m, 양수. |
| `ground.min_height_m` / `max_height_m` | 카메라-평면 수직 거리 후보 범위. 기본 0.08~0.50m. |
| `ground.max_tilt_deg` | 광학 +Y 대비 바닥 법선의 최대 기울기. 기본 45도, 허용 (0, 60]. |
| `ground.min_inlier_ratio` | 하단 샘플 중 평면에 속해야 하는 최소 비율. 기본 0.35, 허용 0.1~1. |
| `publish.depth_image` | depth 이미지 발행 여부. 점군과 CameraInfo는 계속 발행. |

무거우면 `camera.fps`를 낮추거나 `points.pixel_stride: 2`, `publish.depth_image: false`로
호스트 부하를 줄일 수 있다. 원본 해상도 비교에는 stride=1을 유지한다.
장치 내부 StereoDepth 시간 필터는 항상 끄고 device decimation은 1로 유지해 한 프레임의 입력 해상도를 보존한다.

## 토픽과 좌표

| 토픽 | 타입 / 의미 |
|---|---|
| `/point_cloud/points` | `sensor_msgs/PointCloud2`, 원본 XYZ float32, 미터, organized cloud |
| `/point_cloud/points_filtered` | 같은 형식/frame/stamp. 바닥 제거 결과, OFF/검출 실패 시 원본 |
| `/point_cloud/points_bev` | `sensor_msgs/PointCloud2`, 앞차축 기준 3D 점군, BEV 범위 밖은 NaN, ground 토글 반영 |
| `/point_cloud/points_clusters` | `sensor_msgs/PointCloud2`, 차량 좌표, 통과한 원본 XYZ + rgb + cluster_id, compact cloud |
| `/point_cloud/bev_bounds` | `visualization_msgs/Marker`, 지면의 BEV 영역 테두리, transient local |
| `/point_cloud/depth/image_raw` | `sensor_msgs/Image`, rectified-right depth, 16UC1 밀리미터, 0=무효 |
| `/point_cloud/depth/camera_info` | `sensor_msgs/CameraInfo`, 해당 전체 depth 이미지의 실제 내부 파라미터 |
| `/point_cloud/status` | `std_msgs/String`, MEASURING/OPENING/WAITING/STREAMING/STALE/ERROR, transient local |
| `/tf_static` | front_axle_bev → point_cloud_view → point_cloud_optical_frame |

데이터 토픽 QoS는 **Best Effort / Volatile / Keep Last 1**이다. 제공 RViz 설정도 동일하다.
다른 RViz 설정을 사용하면 Fixed Frame=`front_axle_bev`, PointCloud2 topic=`/point_cloud/points_clusters`,
Reliability=`Best Effort`, Color Transformer=`RGB8`로 설정한다.
`bev.frame_id`를 바꾸면 RViz Fixed Frame/Target Frame/Axes도 맞춘다.

원본/바닥 필터 점군 좌표는 **rectified CAM_C optical**: +X 오른쪽, +Y 아래, +Z 전방.
`X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`로 복원하고 내부 파라미터는 매 프레임의
`ImgFrame.getTransformation()`에서 가져온다. 해상도 변경 후 과거 보정값을 재사용하지 않는다.
빈 픽셀/거리 제외 픽셀은 XYZ 모두 NaN으로 유지한다. stride>1일 때도 원래 픽셀 좌표를 쓴다.
이미지/CameraInfo는 stride와 관계없이 전체 depth 크기다.

보기용 `point_cloud_view`는 같은 카메라 원점에서 +X 전방/+Y 왼쪽/+Z 위 방향으로 축만 바꾼다.
차량 원점이나 지면 좌표가 아니며, 카메라 장착 pitch/roll/높이를 보정하지 않는다.
RViz의 기본 Fixed Frame은 별도의 `front_axle_bev`이므로 측정된 장착 변환을 통해
지면 기준으로 표시된다. Fixed Frame을 `point_cloud_view`로 바꾸면 다시 카메라 기준으로 보인다.
ROS 타임스탬프는 수신 시 ROS 시각에서 DepthAI steady-clock 프레임 나이를 빼 추정하며,
원본/필터/BEV/클러스터 점군과 이미지/CameraInfo는 같은 stamp를 사용한다. 이전 프레임을 재발행하거나 누적하지 않는다.

## 기록과 검사

```bash
ros2 topic echo /point_cloud/status --qos-durability transient_local
ros2 bag record /point_cloud/points /point_cloud/points_filtered /point_cloud/points_bev /point_cloud/points_clusters /point_cloud/bev_bounds /point_cloud/depth/image_raw /point_cloud/depth/camera_info /tf_static
colcon test --packages-select point_cloud --event-handlers console_direct+
colcon test-result --verbose
```

테스트는 XYZ 단위/축, NaN, 행 패딩, stride, 거리 문턱값, 변경된 보정값,
잘못된 프레임 및 옵션 조합을 검사한다. 바닥 필터 테스트는 기울어진 바닥/깊이 잡음,
장애물 보존, 거리 범위, ON/OFF, 벽 제외, NaN/퇴화 입력과 옵션 검증을 검사한다.
BEV 테스트는 변환 순서·센서 이동 단위·영역 경계·높이 보존을 검사하고,
27가지 roll/pitch/yaw 조합을 실제 `bev_processor` 회전 구현과 비교한다.
클러스터 테스트는 분리된 장애물, 낮은 바닥 연결점, 높이 이상점/비율, 최소 크기/점 개수,
실행 중 설정 변화, 원래 좌표와 ID 보존, 빈 프레임에서 결과 지우기를 검사한다.
낮은 바닥 띠가 두 콘을 연결하는 장면, 반복 관측된 낮은 점, 밑부분 반경 제한,
중심부만 표시, 높은 이상점 하나로 바닥이 장애물이 되지 않는지를 회귀 검사한다.
반복성 테스트는 고정된 콘과 깜빡이는 연결점, 프레임별 투표, XYZ 흔들림/높이 차이,
기록 만료, 중복 프레임, ON/OFF/재연결 초기화, 잔상 방지를 검사한다. 형태 테스트는 콘에 붙은 선 제거, 가는 다리로 연결된 두 덩어리 분리,
대각선/긴 띠/빈 외곽 제외, 면적 증거, 가상 포인트 미생성, 높이 조건 재검사와 메모리 제한을 검사한다. 실제 FPS, 해상도별 FOV/품질, IR 작동,
RViz 표시와 재연결은 ROS 2와 OAK가 연결된 환경에서 확인해야 한다.

DepthAI 참고: [StereoDepth](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth),
[IR Projectors Control](https://docs.luxonis.com/software-v3/depthai/examples/misc/projectors).

### 덩어리 필터 개발 환경 검증

Mac에서 형태 회귀 테스트는 일반 빌드와 UBSan 빌드로 통과했다. ASan 빌드는 테스트 본문 종료 후
로컬 OpenCV/TBB 종료 단계에서 오류가 발생했고, 프로젝트 코드를 포함하지 않는 단독 OpenCV
connectedComponents 실행에서도 동일하게 재현됐다. 따라서 이 환경의 ASan 전체 실행을 통과한
것으로 기록하지 않는다. ROS 2 노드 전체 빌드와 실제 카메라/RViz 확인은 차량 환경에서 수행한다.
