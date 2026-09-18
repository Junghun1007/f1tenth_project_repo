# roi_lidar

영상에서 지정한 ROI의 스테레오 깊이를 가상 2D LiDAR로 변환하고, **실제 RGB 카메라를 위에서
본 BEV 영상으로 변환한 배경 위에 장애물의 대략적인 위치**를 빨간 점으로 표시한다. RViz는 실행하지 않는다.

처리 순서는 다음과 같다.

1. 시작할 때 `oak_startup`으로 바닥 평면과 CAM_A 높이·roll·pitch를 측정한다.
2. 같은 OAK 장치에서 CAM_B/C 스테레오 깊이와 CAM_A RGB를 함께 받는다.
3. 깊이 영상의 ROI만 샘플링하고 실제 프레임의 내부/외부 보정으로 앞차축 차량 좌표로 변환한다.
4. 측정한 바닥 평면 위 5cm 이하와 최대 높이 밖의 점을 제거한다.
5. 각도별 가장 가까운 수평 거리를 `/roi_lidar/scan`으로 발행한다.
6. RGB를 지면으로 역투영한 BEV 영상 위에 해당 거리의 XY 위치를 표시한다.

클러스터, 물체 ID, 시간 확인/유지, 가상 장애물 점, 빈 영역의 자유 공간 추정은 없다.
빨간 점은 **물체 중심이 아닌 관측된 가장 가까운 표면 위치**다. 각도 bin 때문에 약간의 위치 양자화가 있다.
초록 화살표는 앞차축 지면 원점이며, 화면 위가 전방 +X, 왼쪽이 차량 좌측 +Y다.
거리 표시는 가장 가까운 반환점의 수평 거리다. 회색 격자는 미터 눈금이다.

## 빌드 / 실행

ROS 2 C++, DepthAI C++ 3.6+, OpenCV와 기존 `oak_startup` 측정을 지원하는 OAK가 필요하다.
`point_cloud`의 깊이 입력·보정 라이브러리를 공유하지만 point_cloud/RViz 노드를 실행하지 않는다.
같은 카메라를 여는 camera_driver, bev_processor, depth_lidar, point_cloud 등은 먼저 종료한다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
colcon build --packages-up-to roi_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roi_lidar roi_lidar.launch.py
```

평평한 바닥에서 정지한 상태로 시작한다. 초기 측정 영역에 충분한 바닥이 보여야 한다.
측정 중에는 프리뷰에 MEASURING 상태가 표시된다. 실패하면 원인을 확인하고 노드를 재시작한다.
측정이 끝나면 **RGB BEV 창**과 **ROI 선택 창**이 열린다. ROI 창에서 마우스로 사각형을 드래그한다.
R 키는 전체 영상 ROI로 복원하고, Q/Esc는 노드를 종료한다. 시작은 전체 영상 ROI다.
카메라가 아래로 기울어져 있으면 먼 장애물은 위쪽 행에 나타나므로 좁은 ROI에서 사라질 수 있다.

ROI 선택 창은 기본적으로 **깊이 기준인 rectified-right 흑백 영상**이다.
`preview.camera_image=false`면 깊이 컬러 영상으로 전환하며 흑백 스트림 전송을 중단한다.
RGB BEV 배경은 이 설정과 관계없이 CAM_A로부터 생성된다.

## 거리와 FPS 프로필

| 프로필 | 스테레오 입력 | 요청 FPS | subpixel | 수평 거리 상한 |
|---|---|---:|---|---:|
| `balanced` (기본 YAML) | 400p | 90 | 3-bit ON | 12m |
| `fast` | 400p | 110 | OFF | 기본 YAML의 12m |
| `far` | 800p | 30 | 3-bit ON | 20m |

```bash
ros2 launch roi_lidar roi_lidar.launch.py profile:=fast
ros2 launch roi_lidar roi_lidar.launch.py profile:=far
# 직접 요청값 조절: 프로필보다 우선
ros2 launch roi_lidar roi_lidar.launch.py fps:=80.0 range:=10.0
# GUI 없이 이미지 토픽 사용
ros2 launch roi_lidar roi_lidar.launch.py gui:=false
```

이 값은 **요청 FPS 및 소프트웨어 처리 상한**이며 실측 검출 거리/FPS가 아니다.
Luxonis의 [RVC2 성능 표](https://docs.luxonis.com/overview/toplevel-features/depth)는
Fast Density에서 400p 90FPS, subpixel을 끈 경우 110FPS를 제시한다. 장치 종류, USB,
좌우 검사와 RGB 동시 처리, 센서 노출에 따라 이 패키지의 실제 FPS는 달라진다.
[Subpixel은 원거리 깊이에 유리](https://docs.luxonis.com/software-v3/depthai/examples/stereo_depth/stereo_depth)하지만
속도와 절충이 있다. 실제 장애물 크기·표면·조명·카메라 장착 각도가 사거리를 결정한다.
IR 강도를 높인다고 설정한 거리까지 검출된다는 뜻은 아니다.

기존 3m BEV 크롭은 적용하지 않는다. `range.max_m`는 앞차축 원점에서의 수평 거리이고,
`points.max_depth_m`는 카메라 광축 깊이 상한이다. 먼 거리에서 두 값 모두 확인한다.
깊이 mm 저장 형식에 맞춰 광축 상한은 최대 65.535m까지 허용하지만, 이는 센서 보장 범위가 아니다.
비어 있는 각도 bin은 NaN(미관측)이다. 요청한 ±70도 전체를 카메라가 실제로 관측한다는 뜻도 아니다.

## 실행 중 조절

```bash
# ROI: 영상 좌상단 기준 비율. x+width <= 1, y+height <= 1.
# 전체에서 중앙 80%를 선택하려면 폭을 먼저 줄이고 x를 이동한다.
ros2 param set /roi_lidar roi.width 0.8
ros2 param set /roi_lidar roi.x 0.1
ros2 param set /roi_lidar roi.height 0.6
ros2 param set /roi_lidar roi.y 0.2

# 바닥 평면 위 제거 두께 / 최대 물체 높이
ros2 param set /roi_lidar ground.distance_m 0.05
ros2 param set /roi_lidar height.max_m 1.0
ros2 param set /roi_lidar ground.enabled false
ros2 param set /roi_lidar ground.enabled true

# 거리와 샘플링
ros2 param set /roi_lidar range.max_m 12.0
ros2 param set /roi_lidar points.max_depth_m 20.0
ros2 param set /roi_lidar points.pixel_stride 1

# 깊이 설정: 변경 시 카메라 파이프라인 재시작
ros2 param set /roi_lidar camera.fps 90.0
ros2 param set /roi_lidar camera.resolution 400p
ros2 param set /roi_lidar depth.subpixel true
ros2 param set /roi_lidar depth.ir_dot_projector_intensity 0.7

# 추가 흑백 USB 스트림 없이 깊이 영상에서 ROI 선택
ros2 param set /roi_lidar preview.camera_image false
# 프리뷰 렌더링과 RGB 수신은 각각 별도 속도
ros2 param set /roi_lidar preview.fps 10.0
ros2 param set /roi_lidar preview.rgb_fps 10.0

# 프리뷰를 완전히 끄면 RGB/흑백 스트림도 꺼지고 depth 스캔만 처리
ros2 param set /roi_lidar preview.gui false
ros2 param set /roi_lidar preview.publish false
```

ROI/바닥/높이/거리/scan 설정 변경은 카메라를 재시작하지 않는다. 카메라/깊이 설정,
RGB FPS, GUI/발행 활성화, ROI 카메라 스트림 토글은 파이프라인을 재시작한다.
초기 측정 자세는 재연결 때도 유지한다. 차량 장착 자세나 바닥 경사가 바뀌면 노드를 재시작해 측정한다.
실행 중 변경과 마우스 ROI는 YAML에 자동 저장되지 않는다. `ros2 param dump /roi_lidar`로 저장하거나 YAML에 반영한다.

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `roi.x/y/width/height` | 0/0/1/1 | 선택 영역의 정규화 좌표. 마우스 변경은 네 값을 한 번에 적용. |
| `points.pixel_stride` | 2 | 2×2당 한 픽셀. 1은 작은/먼 장애물 표본 증가, 호스트 비용 증가. |
| `ground.distance_m` | 0.05 | 측정 지면 위 5cm 이하 및 지면 아래 점 제외. 낮은 장애물도 제외될 수 있음. |
| `height.max_m` | 1.0 | 지면 기준 최대 후보 높이. |
| `scan.bins` | 281 | -70~+70도에서 약 0.5도 간격. |
| `scan.min_samples` | 1 | 한 프레임에서 가장 가까운 거리 주변에 필요한 표본 수. 높이면 성긴 장애물도 탈락. |
| `scan.support_distance_m` | 0.15 | min_samples 검사에서 최근접 거리로부터 표본을 인정할 거리 차이. |
| `preview.max_sync_sec` | 0.08 | RGB와 depth의 최대 촬영 시각 차이. 초과하면 overlay 생략, scan은 계속 발행. |
| `input.max_age_sec` | 0.25 | 입력·표시 신선도 제한. 만료 시 빈 scan/표시로 지움. |
| `preview.size_px` | 800 | BEV 프리뷰 가로 크기. 실제 RGB 수신은 640×400. |

일반 depth/IR 옵션은 `config/roi_lidar.yaml`에 있다. subpixel과 extended disparity는 동시에 켤 수 없다.
초기 측정 dot 강도는 별도 `measurement_ir_dot_projector_intensity`다.
프로젝터가 없는 모델이면 초기 측정과 runtime의 두 dot 값을 모두 0.0으로 설정한다.

## 좌표·동기화·한계

launch는 `point_cloud/config/bev_reference.yaml`에서 BEV의 **앞차축 원점, CAM_A X/Y/yaw,
초기 평면 측정 설정**을 가져온다. 검출 범위는 이 패키지의 range 설정으로 정한다.
`bev_config_file:=/absolute/path/bev_config.yaml`로 다른 참조 YAML을 지정할 수 있다.

깊이는 rectified CAM_C 좌표에서 실제 frame 메타데이터와 EEPROM 외부 보정을 거쳐 CAM_A,
앞차축 지면 좌표로 변환된다. RGB도 실제 프레임의 내부/외부 보정을 사용하고, 출력은
undistortion을 요청한다. 한 카메라의 내부 파라미터를 다른 카메라에 재사용하지 않는다.
RGB 지면 BEV와 장애물 XY는 같은 차량 좌표를 사용한다.

배경은 **Z=0 지면 가정의 RGB 역투영**이다. 높이 있는 물체의 RGB 모습은 늘어져 보일 수 있으며
빨간 점은 그 물체의 측정된 XY 표면 위치이므로 늘어진 RGB 실루엣과 꼭 겹치지는 않는다.
RGB 지면 시야 밖은 어둡게 남는다. depth에만 관측된 반환점은 그 위치에도 표시할 수 있다.

RGB 프리뷰와 스캔의 처리 속도는 분리되어 있다. 각 큐는 최신 한 프레임만 보관하며,
RGB/depth 촬영 시각 차이가 허용 범위 안일 때만 최신 스캔을 영상 위에 겹친다.
하드웨어 동기 프레임 묶음은 아니며, 시간 차이를 창에 표시한다. 이동이 빠르면 sync 허용값을
줄이거나 RGB FPS를 높일 수 있지만 표시 빈도/처리 부하와 절충이 생긴다.
오래된 depth, 역순/중복 프레임은 검출하지 않는다. 카메라 오류·재시작 시 즉시 결과를 지운다.

평면은 **시작할 때만 측정하고 고정**한다. 매 프레임 RANSAC이나 IMU/odometry 움직임 보정을
하지 않으므로 경사로·차체 흔들림·먼 거리 깊이 오차가 바닥 잔여점을 만들 수 있다.
물체 분류/군집 추적이 없으므로 잘못된 깊이가 장애물 표면점으로 보일 수 있다.
이 패키지의 목적은 대략적인 위치 확인이며 주행 판단 성능을 검증한 것은 아니다.

## 출력 / 검증

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/roi_lidar/scan` | LaserScan | 현재 프레임의 거리. 원점=앞차축 지면, 동시 관측이므로 time_increment=0. |
| `/roi_lidar/bev_preview` | Image bgr8 | 실제 RGB BEV + 장애물 표면 위치. |
| `/roi_lidar/roi_preview` | Image bgr8 | 실제 ROI를 그린 흑백 또는 깊이 영상. |
| `/roi_lidar/status` | String | 측정/스트리밍/입력 중단/오류, transient local. |

scan과 영상은 Best Effort/Keep Last 1이다. `/scan`은 추정 촬영 시각, 프리뷰는 렌더링 시각을
stamp로 사용한다. GUI가 없는 환경에서는 `gui:=false`와 영상 토픽을 사용한다.
로그에는 요청 FPS와 실제 처리 depth FPS, 호스트 처리 시간, 프레임 나이, ROI 광선 수,
지면/높이 제외 수, 유효 bin 수를 구분해 표시한다. 카메라에서 무효화된 깊이는 복구하지 않는다.

```bash
colcon test --packages-select roi_lidar point_cloud --event-handlers console_direct+
colcon test-result --verbose
ros2 topic hz /roi_lidar/scan
ros2 topic echo /roi_lidar/status --qos-durability transient_local
```

테스트는 기울어진 지면 제거, ROI 경계·stride·패딩, 좌우 축/거리, 원거리 상한,
같은 프레임의 표본 지지 조건, 관측 누락 시 즉시 삭제, RGB/depth 신선도/시각 차이,
RGB 지면 투영과 실제 장애물 overlay 좌표를 확인한다. 실제 장치의 FPS/사거리, ROS 2 전체 빌드,
GUI 드래그 및 주행 중 오차는 차량 PC와 카메라에서 확인해야 한다.
