# depth_lidar

OAK 스테레오 깊이에서 매 프레임 MSAC으로 바닥을 추정하고, 바닥보다 높은 점을
**점유격자에 집계해 연결 영역과 정확한 격자 경계선**을 출력합니다. 벽을 큰 원으로 감싸지
않으며 ㄱ자 모양과 구멍을 유지합니다. 저장된 바닥 파일이나 사전 측정은 사용하지 않습니다.

## 실행

ROS 2, DepthAI C++ 3.x, OpenCV가 필요합니다. 같은 OAK를 사용하는 다른 노드는 종료합니다.

```bash
colcon build --packages-select depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py config_file:="$(ros2 pkg prefix depth_lidar)/share/depth_lidar/config/depth_lidar_roi_test.yaml"
```

시작 즉시 동작합니다. B키 측정은 없습니다. C키는 정렬 좌우 영상 전송을 토글하며 카메라
파이프라인을 재시작합니다. `stereo_preview.gui=false`는 창만 숨깁니다. 테스트 YAML은
레이더/카메라 창을 켜고, 기본 YAML은 창을 끕니다. 빌드·실장치 성능 검증은 별도 필요합니다.

## 처리 과정과 형상

1. 매 프레임 파란 바닥 ROI에서 깊이 유효점을 균일하게 샘플링합니다.
2. MSAC 후보에 법선 방향·카메라 높이·지지점 수/비율 제약을 적용하고 최소 비용 평면을 선택합니다.
   비용은 `sum(min(distance², inlier_distance_m²))`이며, 최소제곱 보정·분포·RMSE 검사도 수행합니다.
3. 초록 탐지 ROI에서 `height > max(min_height_m, noise_scale × RMSE)`인 점을 채택합니다.
   픽셀 히스테리시스 해제 임계값은 `max(inlier_distance_m, 신규 임계값 × release_ratio)`입니다.
4. 샘플 점을 카메라 XY 격자에 직접 집계합니다. 칸별 최소 점 수를 통과하면 관측 1회입니다.
5. 칸별 최근 3프레임 중 2회 확인과 짧은 누락 유지를 적용합니다. 군집 중심 간 일대일 매칭은 없습니다.
6. 확정된 점유 칸을 8방향 연결로 묶고 최소 칸 수·지지점 수보다 작은 영역은 제거합니다.
7. 이웃이 점유되지 않은 칸의 변만 경계로 추출합니다. 노출된 변에는 구멍의 안쪽 경계도 포함됩니다.

격자는 채우기·팽창·convex hull 연산을 하지 않습니다. 벽의 측정 위치가 3m라면 그 근처의
격자 칸을 표시하며 벽 길이에 비례하는 원 반지름을 추가하지 않습니다. 5cm 격자 자체의
양자화로 칸은 관측점보다 각 축에서 최대 약 5cm 넓어질 수 있습니다. 경계는 폐곡선 꼭짓점
목록이 아니라 노출된 **선분 쌍**입니다. 벽/상자의 의미 분류나 안 보이는 뒷면 복원은 하지 않습니다.

## 좌표와 점유 의미

기존 `depth_lidar` 카메라 좌표를 유지합니다. X=카메라 전방, Y=카메라 좌측, Z=카메라 상방이며,
격자 출력 Z=0입니다. 카메라 내부 파라미터로 복원한 점에서 높이 판정만 MSAC 평면을 사용합니다.
**차량/지면 축으로 회전하거나 차량 원점으로 이동하는 변환은 포함하지 않습니다.** 카메라가
기울어져 있으면 이 XY는 지면 수평 좌표와 다릅니다. 차량 경로계획 연결 시 실제 장착 자세를
사용한 좌표 변환과 차량 외형을 고려한 충돌 영역 확장이 필요합니다.
`bev.*`, `sensor.*`, `preview.scale`은 과거 설정 호환용이며 이 격자에 영향을 주지 않습니다.

격자 값은 `100=장애물 관측 또는 유지`, `-1=미확인`입니다. **0=자유 공간은 출력하지 않습니다.**
점이 없거나 바닥이 보이는 것만으로 주행 가능한 공간을 선언하지 않습니다. 가려진 공간,
ROI 밖, 거리 범위 밖, 무효 깊이도 미확인입니다. 이 출력은 장애물 레이어이며 완전한 주행 지도는
아닙니다. 유지가 끝난 칸도 미확인으로 돌아갑니다. 차량 여유 거리/차체 크기는 여기에 포함하지 않습니다.

## 연산량을 제한한 초기값

| 설정 | 값 | 목적/손실 |
|---|---|---|
| `camera.fps`, 해상도 | 30, 400p | 수신·깊이 처리량 제한. 60 FPS보다 시간 해상도 감소 |
| `points.pixel_stride` | 2 | ROI 샘플 수 약 1/4. 작은/얇은 물체는 누락 가능 |
| `grid.resolution_m` | 0.05m | 전방 3m × 좌우 총 6m = 60×120, 7,200칸 |
| `grid.min_points_per_cell` | 2 | 한 점 잡음 억제 |
| `cluster.min_points`, `min_cells` | 6, 2 | 작은 독립 잡음 영역 억제 |
| `ground.max_samples`, `max_iterations` | 1500, 120 | MSAC 거리 평가 상한 180,000/프레임(보정 단계 제외) |
| `nv12.enabled` | false | 부하 측정용 CAM_A USB 스트림 제거 |
| 테스트 `preview.fps`, `stereo_preview.fps` | 15, 10 | 화면 변환/그리기 횟수 제한 |
| `stabilization.hold_sec` | 0.08s | 잠깐 누락 유지, 장기간 위치 누적 방지 |

640×400에서 탐지 ROI는 x=0, y=180, w=640, h=120입니다. stride=2일 때 최대 19,200픽셀을
검사합니다. 바닥 ROI는 x=32, y=180, w=576, h=220이며 샘플 간격은 max_samples에 맞춰 증가합니다.
바닥 제거 이후 집계·칸별 확인·연결 영역·경계 추출은 `O(P+G)`입니다(P=샘플 픽셀, G=전체 칸).
점마다 3D 이웃 그래프를 만들거나 군집 쌍을 정렬하지 않습니다. MSAC 후보의 인덱스 버퍼도 재사용합니다.
대기 루프는 다음 만료 시각이 됐을 때만 격자를 갱신하며, 보조 메시지는 구독자가 있을 때만 구성합니다.
스테레오 preview.fps는 호스트 표시 주기이며 정렬 영상 USB 전송률 제한은 아닙니다. C키로 스트림을
끄면 전송 부하도 줄어듭니다. 값들은 **실측 최적값이 아니라 계산량을 제한한 시작값**입니다.

## 튜닝

- 벽이 끊기면 먼저 깊이 품질과 ROI를 확인하고 `grid.min_points_per_cell: 1`을 비교합니다.
  얇은 물체가 빠지면 `points.pixel_stride: 1`로 복원합니다(ROI 처리량 약 4배).
- 작은 장애물이 제거되면 `cluster.min_cells: 1`, `cluster.min_points: 3` 등으로 낮춥니다.
  잡음 통과도 늘어납니다. 칸 수 조건은 물체 전체 크기가 아니라 관측된 표면의 수평 투영 크기입니다.
- 3cm 격자는 같은 범위에서 20,000칸입니다. 격자를 줄이면 점이 분산돼 칸별 점 수도 감소합니다.
  범위 길이는 해상도의 정수배여야 하며 1축 최대 1,000칸/전체 최대 100,000칸으로 제한합니다.
- 멀리 보려면 `range.max_m`과 `grid.x_max_m`, `grid.y_*`를 함께 설정합니다. 거리 필터는
  카메라 XY 반경이므로 직사각 격자의 모서리 전체가 탐지 범위에 속하지는 않습니다.
- 바닥 추정 실패가 늘면 ROI·깊이 품질·실제 장착 높이/방향을 먼저 확인하고 MSAC 샘플/반복을 늘립니다.
  기본 방향 허용 60도와 높이 0.05~1m는 넓은 초기값입니다. 실제 장착값 근처로 좁히면
  다른 평면 선택을 줄일 수 있으나 테이블 상판 등을 완전히 구분해 주지는 않습니다.
- 1,500점/120회는 이전 3,000점/200회보다 평가 상한이 70% 작지만 약한 바닥 지지점에서 실패할 수 있습니다.

## 안정화와 실패 상태

확인은 군집이 아니라 **칸별 2/3회**입니다. 주황 경계는 현재 관측된 칸, 회색 경계는 이전 관측을
잠깐 유지하는 칸입니다. 같은 군집 내 색이 섞일 수 있으며 군집 개수는 연결 상태에 따라 달라집니다.
유지 시간은 호스트가 해당 프레임 처리를 시작한 시각부터 세므로 USB 지연을 유지 시간에서 빼지 않습니다.
촬영 시각에 따른 오래된 영상·역순 영상 검사는 별도로 유지합니다. 위치/군집 ID 추적은 제공하지 않습니다.
카메라 고정 격자를 사용하므로 차량 이동 보정은 없고 이동 중에는 유지 형상이 짧은 잔상으로 남을 수 있습니다.

평면이 실패하면 `ground_valid=false`와 실패 이유를 발행합니다. 이전 평면으로 새 점을 판정하지 않으며,
이미 확인된 칸만 기존 hold_sec까지 남습니다. 실패가 반복돼도 관측 시각을 갱신하지 않습니다.
유효 평면이 3도/3cm보다 많이 바뀌면 픽셀 이력만 지웁니다. 설정 변경·카메라 재시작·허용치 초과
프레임 공백에서는 격자 이력도 비웁니다. 대기 중 만료 화면은 `WAITING FOR DEPTH`와 마지막 평균 FPS를 표시합니다.

## 출력과 이전 버전에서 전환

| 토픽 | 형식/내용 |
|---|---|
| `/depth_lidar/occupancy` | nav_msgs/OccupancyGrid, 100 또는 -1. 기본 60×120, 해상도 0.05m |
| `/depth_lidar/contours` | visualization_msgs/MarkerArray, 현재/유지 경계를 LINE_LIST 2개로 출력 |
| `/depth_lidar/occupied_cells` | PointCloud2, 점유 칸 중심마다 x/y/z(float32), point_count(uint32), observation_age_sec(float32), 20바이트 |
| `/depth_lidar/ground_valid` | Bool, 현재 바닥 추정 유효 여부. reliable/transient-local |
| `/depth_lidar/ground_status` | String, 지지점/샘플 수·카메라 높이·RMSE 또는 실패 이유 |
| `/depth_lidar/preview` | 레이더 격자와 점유 칸 외곽선 |
| `/depth_lidar/stereo_preview` | 초록=탐지 ROI, 파랑=바닥 추정 ROI. 오른쪽이 깊이 기준 영상 |

OccupancyGrid 인덱스는 `left_row * width + forward_column`입니다. 원점은 (x_min_m,y_min_m),
orientation.w=1이며 각 칸은 해당 좌표부터 resolution_m만큼의 사각형입니다.
`occupied_cells`의 support와 age는 칸별 값입니다. age=0은 이번 프레임 관측, 양수는 유지이며
호스트 관측 이후 경과 시간입니다(촬영~전송 지연은 제외). occupancy에서는 현재/유지를 같은 100으로
표시하므로 나이가 필요하면 occupied_cells를 함께 사용합니다. 메시지 시각과 ground_valid를 함께 확인합니다.
실패/미확인 출력은 장애물 없는 공간을 뜻하지 않습니다. 구독 QoS는 sensor-data best-effort입니다.
Marker ID 0/1은 현재/유지 경계이며 비어지면 DELETE를 발행하고, 프로세스가 중단되면 1초 후 만료됩니다.

기존 원 중심+반지름 `/depth_lidar/obstacles` 토픽은 제거했습니다. 소비자는 새 격자/경계 토픽으로
변경해야 합니다. `cluster.neighbor_distance_m`, `cluster.radius_margin_m`, `cluster.min_radius_m`,
`stabilization.match_distance_m`, `floor.*`가 남은 YAML은 시작 시 거부합니다. 최신 YAML을 사용합니다.
이전 바닥 파일은 읽거나 수정하지 않습니다. 메시지 이름만 바꾸고 기존 radius 파서를 사용하는 것은 호환되지 않습니다.
