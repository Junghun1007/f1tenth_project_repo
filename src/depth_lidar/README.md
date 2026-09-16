# depth_lidar

OAK 스테레오 깊이 영상에서 저장된 바닥 기준보다 가까워진 픽셀을 추출하고,
군집의 평균 위치와 반지름을 레이더 및 ROS 토픽으로 출력하는 독립 ROS 2 패키지입니다.
장애물 추정을 실행할 때마다 바닥을 측정하지 않습니다. **저장된 바닥 파일을 재사용**하며,
사용자가 명시적으로 측정할 때만 기존 파일을 폐기하고 새로 저장합니다.

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

## 바닥 파일을 처음 만드는 방법

1. 카메라를 사용할 자세에 고정하고, 관심 영역에 장애물 없이 바닥만 보이게 합니다.
2. 레이더/스테레오 창에 포커스를 두고 **B**를 누르거나 아래 서비스를 호출합니다.
3. `MEASURING FLOOR n/60`이 끝나고 `READY | SAVED FLOOR`가 표시될 때까지 기다립니다.
4. 이후 장애물을 놓으면 자동으로 검출합니다. 다음 실행에서도 저장 파일을 불러옵니다.

```bash
ros2 service call /depth_lidar/measure_floor std_srvs/srv/Trigger '{}'
ros2 topic echo /depth_lidar/floor_status
```

서비스 응답은 측정 요청 접수를 뜻하며, 완료 여부는 상태 토픽과 화면에서 확인합니다.
새 측정 요청을 처리할 때 **기존 메모리 기준값과 파일을 먼저 삭제**합니다. 측정 도중 다시 B를
누르면 누적값을 버리고 처음부터 새로 측정합니다. 측정이 실패하거나 중단되어도 이전 결과로
돌아가지 않습니다. 새 기준이 준비되기 전에는 장애물 결과를 비웁니다.

기본 저장 경로는 **실행 PC 사용자 홈의 `~/.ros/depth_lidar/floor_reference.bin`**입니다.
설정 YAML과 별도인 바이너리 파일이며, 측정한 픽셀별 평균 깊이(m), 분산 계산용 누적값,
유효 샘플 수, 측정 프레임 수, 장치 ID·깊이 설정·해상도·카메라 내부 파라미터를 저장합니다.
임시 파일을 완전히 기록한 뒤 지정 파일명으로 바꿉니다. 파일 쓰기 실패 시 화면에 표시하며,
새 결과는 현재 프로세스 메모리에서만 사용할 수 있습니다.

- 시작 시 파일이 있으면 호환성을 확인하고 자동으로 불러옵니다. **자동 재측정은 하지 않습니다.**
- 파일이 없거나 읽을 수 없거나 카메라 설정이 다르면 `B: MEASURE` 상태로 대기합니다.
- 저장된 기준은 고정됩니다. 일반 장애물 추정 프레임으로 평균을 갱신하지 않습니다.
- 카메라 높이·기울기나 바닥 형상을 바꿨다면 직접 재측정합니다. 자세 변화는 파일에서 감지하지 않습니다.
- `floor.file`은 실행 중 바꿀 수 없습니다. YAML/실행 인자를 바꾸고 노드를 다시 시작합니다.

## 검출과 군집 위치

바닥 측정은 ROI와 검출 거리 제한에 관계없이 **전체 depth 영상**을 사용합니다. 따라서 측정 후
ROI를 늘리거나 이동할 수 있고, 3m 밖의 바닥 앞에 나타난 3m 이내 물체도 판정할 수 있습니다.
깊이 0은 무효이며 평균에서 제외합니다. 기본 60프레임 중 최소 30회 유효했던 픽셀만 사용합니다.
기준이 없는 픽셀은 물체 판정을 하지 않습니다.

```text
물체 후보 = 바닥 평균 깊이 - 현재 깊이 > max(floor.min_delta_m, floor.noise_scale × 표준편차)
```

현재 ROI 안에서 위 조건과 `range.min_m`~`range.max_m`을 만족하는 픽셀을 추출합니다.
영상의 8방향 인접 픽셀 중 3D 거리가 `cluster.neighbor_distance_m` 이하인 점을 연결합니다.
`cluster.min_points`보다 작은 군집은 제거합니다. stride를 늘리면 샘플 격자상의 이웃을 사용하며
점 수가 줄기 때문에 군집 조건도 함께 조절해야 합니다.

```text
장애물 중심 X = 군집 점들의 전방 좌표 평균
장애물 중심 Y = 군집 점들의 좌측 좌표 평균
반지름 = max(cluster.min_radius_m, 중심에서 가장 먼 군집 점의 수평 거리 + radius_margin_m)
```

좌표는 **바닥과의 차이가 아닌 현재의 원래 깊이**로 계산합니다. `range.offset_m`은 별도의
공통 거리 보정이며, 바닥 학습값에는 적용하지 않고 검출 좌표에만 적용합니다.
반지름 상한으로 큰 장애물을 잘라내지 않습니다. 차량 크기와 경로계획 여유는 사용처에서 더합니다.
카메라 기준 +X는 전방, +Y는 좌측이며 평면상의 군집 출력 Z는 0입니다.

기존 nearest/farthest 선택, 각도 bin 축약, LaserScan 출력은 제거했습니다.
`scan.*`, `cluster.min_bins`, `cluster.max_missing_bins`, `cluster.base_neighbor_distance_m`,
`cluster.angular_neighbor_scale`, `cluster.max_radius_m` 대신 새 YAML을 사용합니다.

## 출력

| 토픽/서비스 | 형식 및 의미 |
|---|---|
| `/depth_lidar/obstacles` | PointCloud2, 군집 하나당 점 하나: x/y/z/radius(float32, m), point_count(uint32) |
| `/depth_lidar/floor_status` | String, 측정/저장/로드 상태. 최신 상태를 유지하는 transient-local 토픽 |
| `/depth_lidar/preview` | 흰색 레이더: 파란 군집 점, 주황 중심·반지름, 바닥 상태, 수신/연산 성능 |
| `/depth_lidar/stereo_preview` | 좌우 정렬 영상과 초록 ROI. 오른쪽이 실제 depth 기준, 왼쪽은 비교 가이드 |
| `/depth_lidar/measure_floor` | Trigger 서비스. 이전 결과를 버리고 새 바닥 측정 요청 |

기준 미준비/측정 중에는 빈 장애물 목록을 발행합니다. 빈 목록 자체는 바닥 학습 완료를
뜻하지 않으므로 경로계획에서는 `floor_status`와 측정 시각도 확인해야 합니다.
카메라 오류 시 상태는 `CAMERA UNAVAILABLE`이 됩니다.

레이더는 군집 중심뿐 아니라 검출된 표면 점들과 반지름을 표시합니다. 원점은 아래 중앙,
위쪽은 전방, 왼쪽은 카메라 좌측입니다. 이 단계에는 앞차축/BEV 좌표 변환을 적용하지 않습니다.

## 화면 및 ROI 조절

- **B**: 바닥 파일 새로 측정. 이전 결과 폐기.
- **C**: 좌우 카메라 전송·화면 켜기/끄기. 파이프라인이 잠깐 재시작되지만 같은 카메라 설정의
  바닥 기준은 유지합니다. 카메라 설정이나 측정 모드를 자동 변경하지 않습니다.
- 창만 숨기려면 `stereo_preview.gui=false`로 설정합니다. 재시작 없이 영상 전송은 유지합니다.

```bash
ros2 param set /depth_lidar stereo_preview.enabled false
ros2 param set /depth_lidar stereo_preview.enabled true
ros2 param set /depth_lidar stereo_preview.gui false
ros2 param set /depth_lidar stereo_preview.gui true
ros2 param set /depth_lidar roi.width_ratio 0.8
ros2 param set /depth_lidar roi.height_ratio 0.30
ros2 param set /depth_lidar roi.bottom_offset_ratio 0.25
```

현재 ROI는 400p에서 x=0, y=180, w=640, h=120입니다. `bottom_offset_ratio`를 늘리면
위로 이동합니다. `height_ratio + bottom_offset_ratio <= 1`이어야 합니다. ROI와 검출 임계값은
실행 중 바꿀 수 있으며 변경값을 유지하려면 YAML에 기록해야 합니다.

## 주요 파라미터

| 파라미터 | 의미 |
|---|---|
| `floor.file` | 영구 바닥 파일 경로. 기본 ~/.ros/depth_lidar/floor_reference.bin |
| `floor.measure_frames` | 명시적 새 측정 시 모을 프레임 수. 기본 60 |
| `floor.min_valid_ratio` | 픽셀별 최소 유효 샘플 비율. 기본 0.50, 최소 2회 |
| `floor.min_delta_m` | 물체 후보의 최소 깊이 차이. 기본 0.05m |
| `floor.noise_scale` | 픽셀별 표준편차에 곱하는 계수. 기본 3.0 |
| `points.pixel_stride` | 검출 ROI의 픽셀 샘플 간격. 바닥 측정은 항상 모든 픽셀 사용 |
| `cluster.min_points` | 군집으로 인정할 최소 픽셀 수. 기본 20 |
| `cluster.neighbor_distance_m` | 인접 후보 픽셀 사이 최대 3D 거리. 기본 0.08m |
| `cluster.radius_margin_m`, `cluster.min_radius_m` | 반지름 여유와 최솟값 |
| `preview.enabled`, `preview.gui`, `preview.fps`, `preview.size_px` | 레이더 발행/창/갱신률/가로 크기 |
| `stereo_preview.enabled`, `stereo_preview.gui`, `stereo_preview.fps` | 좌우 영상 전송/창/호스트 갱신률 |
| `nv12.*` | CAM_A의 동시 USB 부하 측정. 영상 변환·발행·프리뷰 없이 전용 스레드로 수신 |

`floor.measure_frames`, `floor.min_valid_ratio` 변경은 다음 새 측정부터 적용됩니다.
`floor.min_delta_m`, `floor.noise_scale`, 군집 조건은 현재 저장 기준에 즉시 적용됩니다.
`preview.scale`, `bev.*`, `sensor.*`는 이전 설정 호환용으로 남아 있으며 현재 표시에 사용하지 않습니다.
수신 FPS와 객체 처리시간은 계속 표시합니다. 바닥 측정/파일 저장 중의 연산시간에는 그 작업이 포함됩니다.
