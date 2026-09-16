# depth_lidar

OAK 스테레오 깊이 영상을 3차원 점으로 복원하고, **저장한 바닥 평면으로부터의 높이**로
장애물 후보를 구분하는 독립 ROS 2 패키지입니다. 후보 군집의 평균 위치와 반지름을
레이더와 ROS 토픽으로 출력합니다. 픽셀별 배경 깊이 차이와 nearest/farthest 선택은 사용하지 않습니다.

바닥은 사용자가 명시적으로 측정할 때만 새로 저장합니다. 장애물 추정을 실행할 때마다
측정하거나 주행 중 자동으로 평면을 갱신하지 않습니다.

## 빌드 및 실행

ROS 2, DepthAI C++ 3.x, OpenCV와 연결된 OAK 장치가 필요합니다.
같은 OAK를 사용하는 다른 카메라 노드는 종료합니다.

```bash
colcon build --packages-select depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py config_file:="$(ros2 pkg prefix depth_lidar)/share/depth_lidar/config/depth_lidar_roi_test.yaml"
```

테스트용 YAML은 레이더와 스테레오 GUI를 켭니다. 다른 YAML은 `config_file`에 실행 PC의
절대 경로를 전달합니다. 기본 `config/depth_lidar.yaml`은 GUI를 끄고 이미지 토픽을 발행합니다.

## 바닥 평면 파일 만들기

1. 카메라를 사용할 높이·자세로 고정하고 차량을 정지시킵니다.
2. 스테레오 **오른쪽의 파란 측정 ROI** 안에 바닥이 넓게 보이도록 조절하고 장애물을 치웁니다.
3. 창에 포커스를 두고 **B**를 누르거나 아래 서비스를 호출합니다.
4. `MEASURING FLOOR n/60` 이후 `READY | SAVED FLOOR`를 확인합니다.
   상태의 `H`는 추정 카메라 높이, `RMSE`는 평면 적합 오차, `INLIERS`는 평면 지지점 수입니다.
5. 이후 장애물을 놓으면 검출합니다. 다음 실행에서도 저장 파일을 자동으로 불러옵니다.

```bash
ros2 service call /depth_lidar/measure_floor std_srvs/srv/Trigger '{}'
ros2 topic echo /depth_lidar/floor_status
```

서비스 응답은 요청 접수이며, 완료 여부는 상태 토픽/화면에서 확인합니다.
새 측정 요청을 처리하면 **기존 메모리 모델과 파일부터 삭제**합니다. 측정 중 B를 다시 누르면
처음부터 시작합니다. 실패·중단되어도 이전 결과로 복구하지 않으며 유효 평면이 없으면
빈 장애물 목록을 발행합니다. 실패 이유는 로그에 출력합니다.

기본 경로는 **실행 PC의 `~/.ros/depth_lidar/floor_reference.bin`**입니다.
`DLPLANE2` 형식으로 단위 법선·평면 상수, 적합 오차·지지점 수·샘플 수,
측정 프레임 수, 장치 ID·깊이 설정·해상도·내부 파라미터를 저장합니다.
픽셀별 배경 이미지는 저장하지 않습니다. 임시 파일 기록 후 지정 파일명으로 바꿉니다.
저장 실패 시 새 모델은 현재 프로세스 메모리에서만 사용하며 화면·로그에 실패를 표시합니다.

**이전 `DLFLOOR1` 픽셀 오프셋 파일은 호환되지 않습니다. 업데이트 후 B로 한 번 재측정합니다.**

- 파일이 없거나 호환되지 않으면 `B: MEASURE`로 대기하며 자동 측정하지 않습니다.
- 일반 장애물 프레임은 저장된 평면을 바꾸지 않습니다.
- 카메라 높이·기울기 또는 바닥과 카메라의 상대 자세가 바뀌면 직접 재측정합니다.
  장착 자세 변화는 파일 호환성 검사로 감지할 수 없습니다.
- `floor.file` 변경은 YAML을 수정하고 노드를 재시작해야 합니다.

## 바닥 측정과 장애물 구분

측정은 탐지 ROI와 별개인 `floor.measure_roi_*` 영역에서 `floor.fit_pixel_stride` 간격으로
깊이를 모읍니다. 깊이 0과 `floor.fit_max_depth_m` 밖의 값은 제외합니다.
기본 60프레임 중 30회 이상 유효한 픽셀의 평균 깊이를 3D로 복원한 뒤 RANSAC으로 평면을 찾고,
지지점으로 최소제곱 보정합니다. 벽이나 선 형태의 표본을 줄이기 위해 법선 방향·카메라 높이,
지지점 수·비율·분포를 검사합니다. 실패 시 모델을 저장하지 않습니다.

카메라 전방/좌측/상방 좌표의 평면을 `aX+bY+cZ+d=0`으로 표현합니다.
법선 `(a,b,c)`는 길이 1이며 카메라 상방을 향하게 합니다. 이때 `d`는 바닥에서 카메라까지의
수직 거리이고, 점의 `h=aX+bY+cZ+d`는 바닥 위 높이(m)입니다.

```text
장애물 후보 높이: max(floor.min_height_m, floor.noise_scale × 평면 RMSE) <= h <= floor.max_height_m
```

현재 탐지 ROI의 유효 깊이를 3D로 복원하고 위 높이 조건과 수평 거리 범위를 적용합니다.
바닥 아래 점과 높이 문턱 이하의 점은 제외합니다. 측정 당시 깊이가 없었던 픽셀도 현재 유효
깊이가 있으면 같은 평면으로 판정할 수 있습니다. 탐지 ROI 변경만으로 재측정할 필요는 없습니다.

기본 `floor.min_height_m=0.05`는 **바닥 위 5cm 미만의 표면을 후보에서 제외**합니다.
검출할 가장 낮은 장애물에 맞춰 조정합니다. RMSE는 평균 깊이 점들의 평면 적합 오차이며
개별 실시간 깊이 픽셀의 오차를 모두 나타내지는 않습니다.

평면 형상만으로 바닥과 넓은 상판을 완전히 구분할 수는 없습니다. 측정 ROI에는 바닥을 확보하고,
`floor.min_camera_height_m`/`max_camera_height_m` 범위를 실제 설치 높이 주변으로 좁히는 것이 좋습니다.
평평한 바닥에서도 차체 진동·피치/롤로 상대 자세가 바뀌면 저장 평면과 차이가 생깁니다.

## 군집과 좌표

영상의 8방향 이웃 중 3D 거리가 `cluster.neighbor_distance_m` 이하인 후보를 연결합니다.
`cluster.min_points` 미만 군집을 제거합니다. stride를 늘리면 샘플 격자상의 이웃을 사용합니다.

군집 좌표는 바닥에 수직으로 내려놓은 **카메라의 발점**을 원점으로 합니다.
+X는 카메라 전방축을 바닥에 투영한 방향, +Y는 그 바닥에서 좌측, +Z는 바닥 법선입니다.
카메라가 아래로 기울어져 있어도 원래 카메라 전방 깊이를 그대로 수평 거리로 쓰지 않습니다.

```text
장애물 중심 X/Y = 바닥 좌표로 변환한 군집 점들의 X/Y 평균
반지름 = max(cluster.min_radius_m, 중심에서 가장 먼 점의 수평 거리 + radius_margin_m)
```

군집 중심은 관측된 표면점의 평균이며 물체 전체의 기하학적 중심은 아닙니다.
반지름은 관측점들을 덮고 상한으로 잘라내지 않습니다. 가려진 부분과 차량 크기·경로계획 여유는
사용처에서 고려합니다. 출력 Z는 평면상 장애물 중심을 뜻하는 0입니다.
`range.offset_m`은 바닥 좌표의 수평 거리에만 적용하며 높이나 평면 측정값에는 적용하지 않습니다.

기본 `frame_id`는 **`depth_lidar_ground`**입니다. 차량/앞차축 BEV로의 변환과 TF 발행은
포함하지 않습니다. 이전 `depth_lidar` 카메라 좌표와 같은 좌표로 취급하지 마세요.
`scan.*`, 각도 bin 군집 설정, `/depth_lidar/scan`은 사용하지 않습니다.

## 출력과 화면

| 토픽/서비스 | 형식 및 의미 |
|---|---|
| `/depth_lidar/obstacles` | PointCloud2, 군집당 x/y/z/radius(float32, m), point_count(uint32) |
| `/depth_lidar/floor_status` | String, 측정·저장·로드 상태. 최신 상태 유지(transient-local) |
| `/depth_lidar/preview` | 흰색 바닥 좌표 레이더: 파란 군집점, 주황 중심·반지름, 상태·성능 |
| `/depth_lidar/stereo_preview` | 정렬 좌우 영상, 초록 탐지 ROI와 파란 평면 측정 ROI |
| `/depth_lidar/measure_floor` | Trigger, 이전 결과를 버리고 새 바닥 측정 요청 |

평면 미준비/측정 중에는 빈 결과를 발행합니다. 빈 결과만으로 빈 공간을 판단하지 않도록
`floor_status`와 결과 시각도 함께 사용합니다. 카메라 오류 시 상태는 `CAMERA UNAVAILABLE`입니다.
레이더 아래 중앙이 원점, 위가 바닥에서 전방, 왼쪽이 좌측입니다.

**B**는 새 바닥 측정, **C**는 좌우 카메라 전송·창 켜기/끄기입니다.
C로 파이프라인이 재시작되어도 같은 설정의 바닥 모델은 유지합니다.
`stereo_preview.gui`만 바꾸면 영상 전송을 유지한 채 창만 숨기거나 보입니다.
오른쪽 테두리가 실제 depth 기준이고 왼쪽 테두리는 같은 영상 위치의 가이드입니다.

```bash
ros2 param set /depth_lidar stereo_preview.gui false
ros2 param set /depth_lidar stereo_preview.gui true
ros2 param set /depth_lidar roi.height_ratio 0.30
ros2 param set /depth_lidar roi.bottom_offset_ratio 0.25
ros2 param set /depth_lidar floor.min_height_m 0.04
```

640×400 영상 기준 초록 탐지 ROI는 x=0, y=180, w=640, h=120이고,
파란 측정 ROI는 x=64, y=240, w=512, h=140입니다.
`bottom_offset_ratio`가 커지면 위로 이동하며 `height_ratio + bottom_offset_ratio <= 1`이어야 합니다.

## 주요 설정

| 설정 | 기본값과 의미 |
|---|---|
| `floor.measure_frames`, `floor.min_valid_ratio` | 60프레임, 픽셀 유효 비율 0.50 (최소 2회) |
| `floor.measure_roi_width_ratio`, `floor.measure_roi_height_ratio`, `floor.measure_roi_bottom_offset_ratio` | 바닥 측정 영역 0.80 / 0.35 / 0.05 |
| `floor.fit_pixel_stride`, `floor.fit_max_depth_m` | 측정 간격 4픽셀, 최대 광축 깊이 4m |
| `floor.ransac_iterations` | 평면 가설 200회 |
| `floor.inlier_distance_m` | 평면에서 0.02m 이내를 지지점으로 분류 |
| `floor.min_inlier_points`, `floor.min_inlier_ratio` | 최소 100점, 유효 측정점의 0.60 이상 |
| `floor.max_tilt_deg` | 카메라 상방과 평면 법선의 각도 최대 60도 |
| `floor.min_camera_height_m`, `floor.max_camera_height_m` | 추정 카메라 높이 허용 범위 0.05~1.0m |
| `floor.min_height_m`, `floor.max_height_m`, `floor.noise_scale` | 후보 높이 0.05~1.0m, 적합 오차 계수 3.0 |
| `points.pixel_stride` | 탐지 ROI 샘플 간격 1픽셀 |
| `cluster.min_points`, `cluster.neighbor_distance_m` | 최소 20점, 이웃 3D 거리 0.08m |
| `cluster.radius_margin_m`, `cluster.min_radius_m` | 반지름 여유 0.04m, 최소 반지름 0.05m |
| `preview.*`, `stereo_preview.*` | 레이더/좌우 영상 발행·창·갱신률 |
| `nv12.*` | CAM_A 동시 USB 부하 측정용 수신. 변환·발행·프리뷰 없음 |

측정·적합 설정 변경은 **다음 B 측정부터** 적용합니다. 저장 모델이나 진행 중 측정을 바꾸지 않습니다.
`floor.min_height_m`, `floor.max_height_m`, `floor.noise_scale`, 탐지 ROI·거리·군집 설정은
현재 프레임부터 적용됩니다. 변경값을 유지하려면 YAML도 수정해야 합니다.
기존 `floor.min_delta_m`은 제거했습니다. `preview.scale`, `bev.*`, `sensor.*`는 이전 설정
호환용이며 현재 표시에 사용하지 않습니다. 바닥 적합·파일 저장 시간은 처리시간 지표에 포함됩니다.
