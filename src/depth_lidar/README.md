# depth_lidar

OAK 스테레오 깊이를 3D 점으로 복원하고 **매 프레임 MSAC으로 현재 바닥 평면을 추정**합니다.
바닥보다 충분히 높은 점을 군집화해 위치와 반지름을 출력합니다. 저장한 배경 깊이와의 비교,
사전 바닥 측정, 바닥 파일 저장·로드 기능은 제거했습니다.

## 실행

ROS 2, DepthAI C++ 3.x, OpenCV와 연결된 OAK가 필요합니다.
같은 장치를 사용하는 다른 카메라 노드는 종료합니다.

```bash
colcon build --packages-select depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py config_file:="$(ros2 pkg prefix depth_lidar)/share/depth_lidar/config/depth_lidar_roi_test.yaml"
```

테스트 YAML은 레이더와 좌우 카메라 GUI를 켭니다. 다른 파일을 쓰려면 실행 PC의 절대 경로를
`config_file`에 전달합니다. 기본 `config/depth_lidar.yaml`은 GUI를 끄고 이미지 토픽을 발행합니다.
**시작 즉시 추정합니다. B키나 빈 바닥 촬영 절차는 없습니다.** 장애물이 있어도 주변 바닥이 충분히
보이면 추정을 시도합니다. 실제 장치에서의 처리율과 검출 성능은 별도 확인이 필요합니다.

## 이전 설정에서 전환

- `floor.*`, `/depth_lidar/measure_floor`, `/depth_lidar/floor_status`, B키를 제거했습니다.
- 새 YAML의 `ground.*`를 사용합니다. `floor.*`가 남은 YAML로 시작하면 설명과 함께 거부합니다.
- 이전 `floor_reference.bin`은 읽거나 수정하지 않습니다. 삭제하거나 다시 측정할 필요가 없습니다.
- 장애물 토픽의 좌표계와 24바이트 필드 구성은 유지합니다. 평면 유효성은 새 `ground_valid`를 확인합니다.
- 기본 `frame_id`는 계속 `depth_lidar`이며 레이더의 장애물은 주황/회색 원만 표시합니다.

## 한 프레임의 처리

1. 오른쪽 정렬 영상 기준 깊이와 그 프레임의 내부 파라미터로 3D 점을 만듭니다.
   카메라 좌표는 전방 X, 좌측 Y, 상방 Z입니다. 깊이 0은 무효입니다.
2. 파란 `ground.roi_*` 영역을 균일하게 샘플링합니다. `ground.pixel_stride`에서 시작하고
   샘플 격자가 `ground.max_samples`를 넘으면 간격을 늘립니다. 지정 광축 깊이 범위 밖은 제외합니다.
3. 점 3개씩으로 평면 후보를 만들고 방향·카메라 높이 제약을 적용합니다.
   각 후보의 점-평면 거리 r에 대해 `sum(min(r², inlier_distance_m²))`인 **MSAC 비용**을 계산합니다.
   최소 지지점 수·비율을 만족하는 후보 중 비용이 가장 작은 것을 고릅니다.
4. 지지점의 최소제곱 평면으로 보정한 뒤 지지점 분포·수·비율, 방향·높이, RMSE를 다시 검사합니다.
5. 초록 `roi.*` 영역에서 바닥 위 높이 조건과 거리 조건을 통과하는 점들을 군집화합니다.
6. 3프레임 중 2회 확인과 짧은 누락 유지를 적용해 장애물 토픽·레이더로 출력합니다.

현재 프레임에서 실패하면 이전 평면으로 대신 판정하지 않습니다. 픽셀 히스테리시스를 지우고
`ground_valid=false`, `INVALID | 이유`를 출력합니다. 실패 프레임은 군집의 미관측으로 처리하며,
이미 확인된 군집만 마지막 관측부터 `stabilization.hold_sec`까지 회색으로 유지합니다.
실패가 반복돼도 유지 시간을 연장하거나 새 군집을 확인하지 않으며, 만료되면 빈 목록을 출력합니다.

## 높이 판정과 좌표

추정 평면은 `n.x*X + n.y*Y + n.z*Z + d = 0`입니다. n은 단위 법선이며 지정 상방을 향합니다.
카메라는 원점이므로 d는 카메라에서 바닥 평면까지의 수직 거리입니다.
점의 부호 있는 높이는 `h=n·point+d`로 계산합니다.

```text
신규 후보: h > max(ground.min_height_m, ground.noise_scale × 이번 평면 RMSE)
           그리고 h <= ground.max_height_m
유지 후보: h > max(ground.inlier_distance_m, 신규 문턱 × ground.release_ratio)
           그리고 h <= ground.max_height_m
```

기본 신규 문턱은 최소 5cm, 최대 후보 높이는 1m입니다. 5cm 미만의 물체 표면은 신규 후보에서
제외될 수 있습니다. 히스테리시스 유지 문턱도 바닥 지지점 허용 범위 아래로 내려가지 않습니다.
RMSE는 **현재 평면의 지지점 적합 오차**로, 모든 깊이 픽셀의 측정 오차를 나타내지는 않습니다.

후보는 영상 8방향 이웃이면서 3D 거리 `cluster.neighbor_distance_m` 이내인 점끼리 연결합니다.
`cluster.min_points` 미만은 제거합니다. 중심은 카메라 X/Y 평균, 반지름은 중심에서 가장 먼
군집점의 X/Y 거리에 여유를 더한 값입니다. 반지름 상한으로 큰 물체를 잘라내지 않습니다.

**평면은 높이 판정에 사용하고, 출력 X/Y는 고정된 카메라 축을 유지합니다.** 매 프레임 추정된
바닥 축으로 회전시키지 않습니다. 따라서 카메라가 기울어졌다면 출력 X/Y는 바닥 수평 좌표와
다릅니다. 출력 Z=0은 레이더용 2D 군집 표현이며 실제 높이가 0이라는 뜻이 아닙니다.
차량 앞차축/BEV 변환과 TF 발행은 포함하지 않습니다. 차량 크기·가려진 물체 범위·회피 여유는
사용처에서 고려해야 합니다. `range.offset_m`은 후보 판정 뒤 기존 카메라 XY 거리 보정에 적용합니다.

## 먼저 맞출 설정

| 설정 | 기본값 | 의미와 조절 방향 |
|---|---|---|
| `ground.roi_width_ratio`, `height_ratio`, `bottom_offset_ratio` | 0.90 / 0.55 / 0.0 | 바닥이 넓게 보이는 추정 영역. 장애물 탐지 ROI와 별개 |
| `ground.min_camera_height_m`, `max_camera_height_m` | 0.05 / 1.0m | 실제 카메라 설치 높이 주변으로 좁혀 다른 평면 선택을 줄임 |
| `ground.reference_up_x/y/z` | 0 / 0 / 1 | 카메라 전방/좌측/상방 축으로 표현한 예상 바닥 법선. 내부에서 정규화 |
| `ground.max_tilt_deg` | 60도 | 예상 법선에서 허용하는 편차. 실제 장착 기울기에 맞춰 방향과 범위를 설정 |
| `ground.inlier_distance_m` | 0.02m | 바닥 지지점의 평면 거리 허용치. 크게 하면 잡음뿐 아니라 낮은 물체도 흡수 가능 |
| `ground.min_inlier_points`, `min_inlier_ratio` | 100 / 0.35 | 유효 샘플 중 최소 바닥 지지 조건. 높이면 엄격해지지만 가림에 취약 |
| `ground.min_spread_m` | 0.05m | 지지점이 선처럼 모이지 않도록 평면 내 두 번째 주축 표준편차의 최솟값 |
| `ground.max_rmse_m` | 0.015m | 적합 오차 상한. 넘으면 바닥 판정 실패 |
| `ground.min_height_m`, `max_height_m` | 0.05 / 1.0m | 장애물 후보의 바닥 위 높이 범위 |
| `ground.noise_scale`, `release_ratio` | 3.0 / 0.60 | 평면 오차 계수, 픽셀 유지 문턱 비율 |
| `ground.max_samples`, `max_iterations` | 3000 / 200 | 연산량 제한. 늘리면 더 많은 표본·후보를 보지만 처리 부하 증가 |
| `ground.min_depth_m`, `max_depth_m` | 0.10 / 4.0m | 추정용 광축 깊이 범위. 장애물 검출 거리와 별개 |

기본 높이·각도 범위는 설치값을 모르는 상태의 넓은 범위입니다. MSAC은 바닥이라는 의미를
스스로 알지 못합니다. 큰 상자 윗면도 조건을 만족하면 선택될 수 있으므로 **실제 설치 높이와
예상 법선 제약을 맞추고 바닥 지지 영역을 확보**해야 합니다. 바닥이 많이 가려지거나 계단·여러
높이의 바닥이 섞이면 단일 평면 가정이 맞지 않을 수 있습니다.

## 안정화와 실패 상태

`stabilization.*`의 2/3 확인, 80ms 유지, 카메라 XY 중심 15cm 이내 일대일 연결은 유지합니다.
신규 관측은 주황 원, 누락 유지는 회색 원입니다. 좌표 글씨·개별 점·군집 중심 십자는 표시하지 않습니다.
`stabilization.enabled=false`는 픽셀 히스테리시스와 군집 안정화를 끕니다. 평면 추정은 계속 수행합니다.

군집 갱신과 프레임 대기 중 만료 판정은 모두 호스트 단조 시계를 사용합니다. 마지막으로 해당
군집을 관측한 프레임의 호스트 처리 시작 시점부터 80ms를 세므로 USB 전송 지연이 유지 시간을
미리 소모하지 않습니다. 촬영 시각은 별도로 오래된 영상·역순 영상·프레임 공백 검사에 사용합니다.
대기 중 만료 화면에는 `WAITING FOR DEPTH`를 표시하고 마지막 측정 FPS 평균을 유지합니다.
이 화면의 빈 목록은 새 프레임에서 장애물이 없다고 판정한 것이 아니라 기존 관측이 만료된 것입니다.

이전 유효 프레임과 법선이 `ground.reset_history_angle_deg`(3도)보다 많이 바뀌거나 카메라 높이가
`ground.reset_history_height_m`(3cm)보다 많이 달라지면 픽셀 히스테리시스만 비우고 새 유효 평면으로
높이를 다시 판정합니다. 군집 좌표는 카메라 XY이므로 이 변화만으로 군집 확인 이력을 지우지는
않습니다. 평면을 시간 평균하거나 이전 평면을 대체 사용하지 않습니다.
탐지/추정 설정 변경·카메라 재시작·허용치를 넘는 프레임 공백에는 군집 이력까지 비웁니다.

원이 완전히 사라지는 것과 주황/회색 전환은 구분해야 합니다. 주황/회색 전환은 현재 관측과
일시 유지의 차이입니다. 계속 사라진다면 `ground_status`의 실패 이유와 실제 처리 FPS를 확인합니다.
`VALID`가 유지되면서 잠깐 누락되는 경우에는 `hold_sec`를 0.08에서 0.12로 늘려 비교할 수 있지만,
주행 중 과거 위치를 더 오래 유지하게 됩니다. `hold_sec`는 `max_frame_gap_sec` 이하여야 합니다.

영상이 없을 때에도 수신 루프에서 유지 군집을 시간으로 만료시킵니다. 유효 영상 수신 공백 또는
영상 나이가 `stabilization.max_frame_gap_sec`(200ms)를 넘으면 이력 초기화/오래된 영상 제외를
적용합니다. `DEPTH STALE`이나 `CAMERA UNAVAILABLE`에서는 유효성 false와 빈 결과를 출력합니다.
유지 군집은 마지막 카메라 좌표를 사용하며 **차량 이동 보정은 없습니다**.

## 토픽과 화면

| 토픽 | 의미 |
|---|---|
| `/depth_lidar/obstacles` | PointCloud2: x/y/z/radius(float32), point_count(uint32), observation_age_sec(float32). point_step=24 |
| `/depth_lidar/ground_valid` | Bool, 현재 바닥 추정 유효 여부. reliable/transient-local |
| `/depth_lidar/ground_status` | String, VALID + 지지점/샘플 수·높이·RMSE 또는 실패 이유. reliable/transient-local |
| `/depth_lidar/preview` | 군집 원과 레이더 격자, 평면 상태, 수신/연산 성능 |
| `/depth_lidar/stereo_preview` | 정렬 좌우 영상. 초록=탐지 ROI, 파랑=바닥 추정 ROI. 오른쪽이 실제 depth 기준 |

**빈 장애물 목록과 추정 실패는 같은 뜻이 아닙니다.** 사용처는 유효성과 데이터 수신 시각을 함께
확인해야 합니다. `ground_valid=true`도 ROI 밖이나 무효 깊이 영역까지 빈 공간임을 보장하지 않습니다.
`ground_valid=false`일 때 남아 있는 군집은 이전 관측의 단기 유지이며 현재 검출 결과가 아닙니다.
`observation_age_sec=0`은 해당 depth 프레임에서 관측, 양수는 마지막 관측을 유지 중임을 뜻합니다.
유지군집의 `point_count`도 마지막 관측값입니다. 경과 시간은 호스트 처리 시작 시각 기준이며
촬영 이후의 전송 지연은 포함하지 않습니다. 프레임 중단 시 만료 갱신에도 같은 단조 시계를 사용합니다.

C키로 좌우 영상 전송·창을 토글합니다. 창만 숨기려면 `stereo_preview.gui=false`를 사용합니다.
640×400 기본값에서 초록 ROI는 x=0, y=180, w=640, h=120이고 파란 ROI는 x=32, y=180,
w=576, h=220입니다. `height_ratio + bottom_offset_ratio <= 1`이어야 합니다.

```bash
ros2 topic echo /depth_lidar/ground_valid
ros2 topic echo /depth_lidar/ground_status
ros2 param set /depth_lidar ground.min_height_m 0.04
ros2 param set /depth_lidar ground.roi_height_ratio 0.60
ros2 param set /depth_lidar stereo_preview.gui false
```

`ground.min_height_m`은 `ground.inlier_distance_m`보다 커야 하고, `ground.max_rmse_m`은
`ground.inlier_distance_m` 이하여야 합니다. 설정 변경은 다음 처리 프레임부터 적용되며 유지하려면
YAML에도 기록해야 합니다. `preview.scale`, `bev.*`, `sensor.*`는 이전 호환용으로 현재 표시에는
사용하지 않습니다. NV12 수신 부하 측정은 유지하며 처리시간에는 MSAC·군집·안정화 연산이 포함됩니다.
