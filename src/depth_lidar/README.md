# depth_lidar: 초기 자세를 고정한 가상 2D LiDAR

차선을 보기 위해 아래로 기울인 OAK 카메라에서 전방 장애물의 대략적인 위치를 얻는다.
시작할 때만 `bev_processor`와 공용인 `oak_startup`으로 roll/pitch/높이를 측정하고,
이후에는 고정 변환 → 높이 필터 → 각도별 최단 거리만 처리한다.
주행 중 평면 추정, 자세 갱신, 점유격자, 군집화, 외곽선 계산은 수행하지 않는다.

## 실행

DepthAI C++ 3.6+, ROS 2, OpenCV와 보정된 IMU가 있는 OAK가 필요하다.
공용 패키지가 추가되었으므로 기존 설치에서도 의존 패키지를 함께 빌드한다.

```bash
colcon build --packages-up-to depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py
# GUI/ROI 확인용
ros2 launch depth_lidar depth_lidar.launch.py config_file:=$(ros2 pkg prefix --share depth_lidar)/config/depth_lidar_roi_test.yaml
```

1. 평평한 바닥에 차를 정지시키고 초기 측정 ROI에 충분한 바닥이 보이게 한다.
2. 초기 측정은 RGB(CAM_A)에 정렬된 1280×800 depth와 IMU를 사용한다.
   IMU 정지 여부, 평면 방향/잔차, 여러 프레임의 높이·법선 안정성을 확인한다.
   기본 `startup.attitude_source: depth`는 평면에서 자세를 얻고 IMU로 확인한다.
3. 성공하면 초기 장치 연결을 닫고 **같은 장치 ID**로 저해상도 depth 파이프라인을 연다.
   측정한 roll/pitch/높이는 프로세스 종료까지 고정한다. 이후 IMU 수신도 하지 않는다.
4. `/depth_lidar/ready`가 true가 된 뒤 스캔을 사용한다.
   초기 측정 실패 시 원인을 `/depth_lidar/status`와 로그에 표시하고 검출을 시작하지 않는다.
   정지 상태와 ROI를 확인한 후 노드를 재시작한다. 이동 중 자동 재측정은 없다.

`startup.*`는 실행 중 변경할 수 없다. 다른 파라미터를 변경하면 runtime 카메라
파이프라인과 짧은 거리 보존 이력을 재시작하지만 초기 자세는 유지한다.
C 키는 runtime 카메라 창/USB 스트림을 켜고 끈다. 기본 실행은 GUI를 사용하지 않는다.

## 좌표와 처리

초기 측정의 기준은 CAM_A optical, runtime depth는 rectified CAM_C optical이다.
장치 EEPROM의 오른쪽 rectification 회전 역변환과 CAM_C→CAM_A 외부 보정을 결합한다.
최종 변환은 다음과 같다.

```text
p_vehicle = T_vehicle_CAM_A · T_CAM_A_CAM_C · R_rectified_right^T · p_rectified_right
T_vehicle_CAM_A.rotation = Rz(yaw) · Ry(pitch_down) · Rx(roll) · optical_to_FLU
```

`frame_id: front_axle`는 앞차축 아래 지면을 원점으로 하는 +X 전방/+Y 왼쪽/+Z 위
좌표계다. `sensor.x_m/y_m/yaw_deg`는 **CAM_A의** 수평 위치와 yaw이며 실제로 적용된다.
높이와 roll/pitch는 초기 측정값이다. 기존 YAML의 센서 위치가 depth 카메라 기준이었다면
CAM_A 기준으로 바꾼다. TF는 자동 발행하지 않으며, 다른 노드와 연결할 때 실제 원점·축을
동일하게 설정해야 한다. 이름만 바꿔서 다른 좌표계를 만들 수는 없다.

픽셀 `(u,v)`와 광축 깊이 `D`로 `(D(u-cx)/fx, D(v-cy)/fy, D)`를 복원한다.
고정 변환 뒤 `height.min_m..height.max_m`에 들어가는 점만 사용한다.
수평 거리 `hypot(X,Y)`와 방향 `atan2(Y,X)`를 구하고 각도 bin마다 가장 가까운 값을 남긴다.
깊이 이웃이 없는 고립점과 표본 수가 부족한 bin은 제외한다. 평균 거리나 군집 중심을
사용하지 않으므로 긴 벽을 큰 원으로 앞쪽까지 확장하지 않는다.

각 픽셀의 회전된 광선과 높이 조건을 만족하는 raw-depth 구간을 미리 계산한다.
매 프레임에는 깊이 범위 검사 → 필요한 점의 XY 계산 → bin 최솟값 갱신을 수행한다.
전체 3D 포인트클라우드를 만들거나 정렬하지 않는다. 연산은 표본 수에 선형이며,
30 FPS/400p/stride 2/141 bins를 기본으로 둔다. 실제 성능은 하드웨어에서 확인해야 한다.
로그의 `project+filter ms`는 호스트 필터·변환·scan hold 시간이며 OAK stereo 연산,
USB 수신 대기, ROS 발행, 프리뷰 비용을 포함하지 않는다.

## 주요 설정

| 설정 | 의미 / 조정 방향 |
|---|---|
| `startup.roi_*` | 초기 1280×800 CAM_A 정렬 depth의 바닥 영역. runtime `roi.*`와 별개다. |
| `startup.minimum_height_m/maximum_height_m` | 예상 카메라 장착 높이 허용 범위. 기본 0.10~0.40m. |
| `startup.ir_dot_projector_intensity` | 기본 0. Pro 장치에서 바닥 depth가 부족하면 0~1 범위로 설정. |
| `startup.timeout_sec` | 안정된 초기 표본을 기다릴 한도, 기본 45초. |
| `height.min_m` | 바닥으로 제외할 높이. 기본 5cm. 높일수록 바닥 오검출은 줄지만 낮은 물체를 놓친다. |
| `height.max_m` | 검출할 높이 상한. 기본 40cm. 실제 차체 충돌 높이에 맞춘다. |
| `points.pixel_stride` | 기본 2. 3~4로 높이면 표본과 연산이 감소하지만 작은 물체를 놓치기 쉽다. |
| `roi.*` | 기본 전체 영상. 기울어진 카메라에서는 같은 높이가 거리마다 다른 행에 보인다. 좁히면 사각이 생길 수 있다. |
| `scan.bins` | 기본 -70~+70도에 141개(1도). 방향 해상도를 낮추려면 줄인다. |
| `scan.min_points_per_bin` | 기본 2. 높이면 잡음과 작은 장애물 모두 더 많이 제외된다. |
| `filter.min_neighbors/neighbor_delta_m` | 기본 이웃 1개, raw-depth 차이 8cm 이내. 작은 물체가 사라지면 먼저 이 필터와 stride를 확인한다. |
| `scan.hold_sec` | 기본 80ms. 미관측 bin만 짧게 보존한다. 0이면 현재 프레임만 사용한다. |
| `input.max_age_sec` | 기본 200ms. 오래된 입력/입력 단절 시 결과를 비운다. |

높이 필터는 초기 바닥 기준이며 지면의 경사 변화나 차체 pitch/roll 변화는 보정하지 않는다.
카메라가 내려다보는 각도/FOV 밖에 있는 물체는 좌표 변환으로 복구할 수 없다.
수평의 얇은 단일 레이저 면 대신 **높이 구간의 가장 가까운 표면**을 투영한 가상 스캔이다.

## 출력

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/depth_lidar/scan` | `sensor_msgs/LaserScan` | 고정 차량 좌표계 방향별 수평 거리. 부재/무효/FOV 밖은 NaN. |
| `/depth_lidar/scan_points` | `sensor_msgs/PointCloud2` | 유효 bin의 x/y, z=0, `observation_age_sec`. 구독 시 생성. |
| `/depth_lidar/status` | `std_msgs/String` | 초기 측정/준비/입력 장애 상태. transient-local. |
| `/depth_lidar/ready` | `std_msgs/Bool` | 초기 측정 성공 후 유효한 최신 depth를 처리 중인지. 검출점 개수와 별개. |
| `/depth_lidar/startup_pose` | `std_msgs/String` | 고정 CAM_A roll/pitch/높이, 장치 ID, 자세 출처. 진단용, TF 아님. |
| `/depth_lidar/preview` | `sensor_msgs/Image` | 흰 레이더, 주황=현재점, 회색=짧게 유지한 점. |
| `/depth_lidar/stereo_preview` | `sensor_msgs/Image` | 좌/우 정렬 영상과 초록 샘플링 ROI. 왼쪽 ROI는 위치 안내이며 정확한 대응점 아님. |

스캔 원점은 차량 원점이므로 카메라 원점과 다르고, 여러 높이의 반환값을 합친다.
NaN을 최대거리 또는 통과 가능한 빈 공간으로 치환하지 않는다. 특히 이 스캔을 이용한
자유공간 ray-clearing은 별도로 가시성을 처리해야 한다. `LaserScan` 자체에는 점별 age가
없다. hold를 끄거나 `scan_points.observation_age_sec`를 함께 사용한다.
유지 시간은 호스트가 관측한 시각부터 계산하며 USB 지연은 age 필드에 포함하지 않는다.
현재 프레임 발행 시 header는 추정 촬영 시각, 입력 대기 중 만료 갱신은 발행 시각이다.

기존 `ground.*`, `floor.*`, `grid.*`, `cluster.*`, `stabilization.*`, `bev.*`,
`preview.scale`, `scan.range_selection`은 제거했다. 해당 설정 파일은 오류로 거부한다.
기존 occupancy/contours/cells/ground_status/ground_valid 토픽도 제거했으므로 소비 노드는
새로운 scan/scan_points/ready 인터페이스로 변경해야 한다.

같은 OAK를 `camera_driver` 또는 `bev_processor`가 이미 열고 있으면 독립 depth_lidar와
동시에 사용할 수 없다. 이번 변경은 초기 측정 **알고리즘 공유**이며 장치/스트림의 동시
소유 통합은 아니다. BEV와 동시 운용하려면 한 파이프라인에서 영상을 공유하도록 연결해야 한다.
