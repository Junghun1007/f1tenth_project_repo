# point_cloud

OAK의 **한 depth 프레임을 원본 해상도의 XYZ 점군으로 표시**하는 ROS 2 패키지다.
해상도와 IR 조명을 바꾸면서 바닥·벽·장애물의 깊이 품질을 비교한다.
원본 점군과 선택적으로 바닥을 제거한 점군을 함께 발행한다.
시작 자세 측정, 차량 좌표 변환, 각도별 최단점 축약, 군집화, 시간 누적은 하지 않는다.

여기서 원본은 **StereoDepth가 계산한 depth의 유효 픽셀 전체**를 뜻한다. 센서 RAW 영상이나
필터 이전 disparity가 아니다. 스테레오 매칭·신뢰도·좌우 일치 검사에 의해 무효화된 점은
복구하지 않는다. 기본값은 추가 median/공간/점 잡음/홀 채우기 필터를 끄며, 설정으로 켤 수 있다.

## 빌드와 실행

기존 프로젝트와 같은 ROS 2 C++ 환경 및 **DepthAI C++ 3.6+**가 필요하다.
CAM_B/CAM_C 스테레오 카메라를 사용한다. 기본 dot 강도 0.5는 이를 지원하는 OAK Pro용이다.
같은 OAK를 점유하는 `depth_lidar`, `ir_camera_driver`, 다른 카메라 노드는 먼저 종료한다.

```bash
cd /path/to/f1tenth_project_repo
colcon build --packages-select point_cloud --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch point_cloud point_cloud.launch.py
```

RViz가 함께 열리고 기본적으로 바닥 제거 점군을 표시한다.
원점을 카메라에 둔 3D 점군을 회전/확대해 볼 수 있다.
색은 전방 거리 0~4m를 나타내며, 이 색 범위가 점을 잘라내지는 않는다.
`Decay Time: 0`은 최신 점군만 표시한다. Axes는 카메라 원점이고 지면 높이 표시가 아니다.
장치 연결이나 설정 적용 오류는 터미널과 `/point_cloud/status`에 표시한다.

```bash
# 고해상도 비교
ros2 launch point_cloud point_cloud.launch.py resolution:=800p fps:=30.0 dot:=0.5

# Dot 프로젝터를 지원하지 않는 OAK 또는 IR OFF 비교
ros2 launch point_cloud point_cloud.launch.py resolution:=400p dot:=0.0

# 사용자 설정 파일 / RViz 없이 발행
ros2 launch point_cloud point_cloud.launch.py config_file:=/absolute/path/point_cloud.yaml rviz:=false
```

`resolution`, `fps`, `dot`, `flood`, `ground` launch 인자를 생략하면 YAML 값이 적용된다.
설정 파일은 `config/point_cloud.yaml`이다. 소스 YAML을 수정했다면 다시 빌드하거나
`config_file`에 수정한 파일의 절대 경로를 전달한다.

## 바닥 제거 켜기 / 끄기

```bash
# 기본 ON. 처음부터 원본으로 보기
ros2 launch point_cloud point_cloud.launch.py ground:=false

# 실행 중 ON / OFF (카메라 재시작 없이 다음 프레임부터 적용)
ros2 param set /point_cloud ground.enabled true
ros2 param set /point_cloud ground.enabled false

# 바닥 두께가 남을 때 제거 허용 거리 조절 (미터)
ros2 param set /point_cloud ground.distance_m 0.03
```

RViz 왼쪽 Displays에서 **Ground-filtered point cloud**가 기본 체크되어 있다.
**Original point cloud (includes ground)**를 체크하고 필터 점군을 해제하면 원본을 비교할 수 있다.
두 항목을 동시에 체크하면 원본의 바닥도 겹쳐 보인다. `ground.enabled=false`면
필터 토픽도 원본을 그대로 표시한다. 원본 `/point_cloud/points`는 항상 보존된다.

현재 프레임의 하단 절반에서 최대 2,000점을 샘플링하고 RANSAC으로 바닥 평면을 찾는다.
카메라 아래 방향과의 각도, 카메라와 평면의 수직 거리, 지지점 비율/분포로 후보를 제한하여
수직 벽을 제외한다. 기본 제거 범위는 광축 거리 3m 이내, 바닥 평면에서 ±2cm이다.
범위 밖 점은 유지한다. 원점·축과 카메라 기울기는 바꾸지 않으며 BEV 좌표 변환은 하지 않는다.
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
`ground.*`만 변경하면 카메라를 재시작하지 않고 호스트 필터에 적용한다. 잠시 점군 출력이
멈추며 IR 강도도 매번 재적용한다. 파라미터 서비스의 성공은 값 검증/저장 성공을 뜻한다.
실제 장치 적용 성공은 `/point_cloud/status`의 `STREAMING`과 새 프레임 로그로 확인한다.
지원하지 않는 조명/모드 조합은 오류로 표시하며 설정을 몰래 낮추지 않는다.
오류 상태에서도 파라미터를 수정할 수 있고, 2초 간격으로 연결을 재시도한다.
`device_id`, `frame_id`, `view_frame_id`는 시작할 때만 설정한다.

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
두 점군 토픽을 모두 구독하면 XYZ 메시지 데이터량은 최대 두 배가 된다.
USB에서는 16비트 depth를 수신하고 호스트가 XYZ로 변환하므로 XYZ 수치가 USB 사용량은 아니다.
ROS 직렬화, RViz, depth 이미지 발행 비용이 더해진다. 로그의 FPS는 카메라 설정값이 아닌
처리·발행한 fresh frame의 실제 속도이며, 큐는 최신 1개를 유지하고 밀린 프레임은 버린다.

처음에는 동일한 장면/조명/스테레오 설정에서 400p와 800p를 비교한다.
작은 장애물의 점 밀도, 바닥의 두께/흔들림, 물체 경계, 유효 점 비율, FPS를 함께 본다.
로그의 `host`는 XYZ 복원·바닥 필터·메시지 생성/발행 호출 시간이며 카메라 연산과 RViz 렌더링은 제외한다.
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
| `ground.enabled` | 바닥 제거 ON/OFF. 기본 true, 원본 토픽에는 영향 없음. |
| `ground.distance_m` | 평면에서 제거할 거리. 기본 0.02m, 허용 (0, 0.10]. |
| `ground.max_depth_m` | 평면 추정 및 제거 범위의 광축 Z 상한. 기본 3m, 양수. |
| `ground.min_height_m` / `max_height_m` | 카메라-평면 수직 거리 후보 범위. 기본 0.08~0.50m. |
| `ground.max_tilt_deg` | 광학 +Y 대비 바닥 법선의 최대 기울기. 기본 45도, 허용 (0, 60]. |
| `ground.min_inlier_ratio` | 하단 샘플 중 평면에 속해야 하는 최소 비율. 기본 0.35, 허용 0.1~1. |
| `publish.depth_image` | depth 이미지 발행 여부. 점군과 CameraInfo는 계속 발행. |

무거우면 `camera.fps`를 낮추거나 `points.pixel_stride: 2`, `publish.depth_image: false`로
호스트 부하를 줄일 수 있다. 원본 해상도 비교에는 stride=1을 유지한다.
시간 필터는 항상 끄고 device decimation은 1로 유지해 한 프레임의 입력 해상도를 보존한다.

## 토픽과 좌표

| 토픽 | 타입 / 의미 |
|---|---|
| `/point_cloud/points` | `sensor_msgs/PointCloud2`, 원본 XYZ float32, 미터, organized cloud |
| `/point_cloud/points_filtered` | 같은 형식/frame/stamp. 바닥 제거 결과, OFF/검출 실패 시 원본 |
| `/point_cloud/depth/image_raw` | `sensor_msgs/Image`, rectified-right depth, 16UC1 밀리미터, 0=무효 |
| `/point_cloud/depth/camera_info` | `sensor_msgs/CameraInfo`, 해당 전체 depth 이미지의 실제 내부 파라미터 |
| `/point_cloud/status` | `std_msgs/String`, OPENING/WAITING/STREAMING/STALE/ERROR, transient local |
| `/tf_static` | RViz 보기용 카메라 축 변환 |

데이터 토픽 QoS는 **Best Effort / Volatile / Keep Last 1**이다. 제공 RViz 설정도 동일하다.
다른 RViz 설정을 사용하면 Fixed Frame=`point_cloud_view`, PointCloud2 topic=`/point_cloud/points_filtered`,
Reliability=`Best Effort`, Color Transformer=`AxisColor`로 설정한다.
`view_frame_id`를 바꾸면 RViz Fixed Frame/Target Frame도 맞춘다.

점군 좌표는 **rectified CAM_C optical**: +X 오른쪽, +Y 아래, +Z 전방.
`X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`로 복원하고 내부 파라미터는 매 프레임의
`ImgFrame.getTransformation()`에서 가져온다. 해상도 변경 후 과거 보정값을 재사용하지 않는다.
빈 픽셀/거리 제외 픽셀은 XYZ 모두 NaN으로 유지한다. stride>1일 때도 원래 픽셀 좌표를 쓴다.
이미지/CameraInfo는 stride와 관계없이 전체 depth 크기다.

보기용 `point_cloud_view`는 같은 카메라 원점에서 +X 전방/+Y 왼쪽/+Z 위 방향으로 축만 바꾼다.
차량 원점이나 지면 좌표가 아니며, 카메라 장착 pitch/roll/높이를 보정하지 않는다.
따라서 아래로 기울인 카메라에서는 바닥도 기울어져 보이는 것이 정상이다.
ROS 타임스탬프는 수신 시 ROS 시각에서 DepthAI steady-clock 프레임 나이를 빼 추정하며,
원본/필터 점군과 이미지/CameraInfo는 같은 stamp를 사용한다. 이전 프레임을 재발행하거나 누적하지 않는다.

## 기록과 검사

```bash
ros2 topic echo /point_cloud/status --qos-durability transient_local
ros2 bag record /point_cloud/points /point_cloud/points_filtered /point_cloud/depth/image_raw /point_cloud/depth/camera_info /tf_static
colcon test --packages-select point_cloud --event-handlers console_direct+
colcon test-result --verbose
```

테스트는 XYZ 단위/축, NaN, 행 패딩, stride, 거리 문턱값, 변경된 보정값,
잘못된 프레임 및 옵션 조합을 검사한다. 바닥 필터 테스트는 기울어진 바닥/깊이 잡음,
장애물 보존, 거리 범위, ON/OFF, 벽 제외, NaN/퇴화 입력과 옵션 검증을 검사한다. 실제 FPS, 해상도별 FOV/품질, IR 작동,
RViz 표시와 재연결은 ROS 2와 OAK가 연결된 환경에서 확인해야 한다.

DepthAI 참고: [StereoDepth](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth),
[IR Projectors Control](https://docs.luxonis.com/software-v3/depthai/examples/misc/projectors).
