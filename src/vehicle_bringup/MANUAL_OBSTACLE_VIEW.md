# 수동주행: 차선 BEV + Depth 장애물

기존 차선 BEV를 **가로 120cm × 전방 300cm**로 유지한다. 차선 모델 입력은 120×300이고,
프리뷰의 좌우 30px 검은 여백은 기존 차선 표시용 여백이다. 장애물은 실제 BEV 영역
X=0~3m, Y=-0.6~+0.6m 안에서만 표시하며 여백을 추가 감지 영역으로 해석하지 않는다.

## 빌드 및 실행

차량 PC의 저장소 루트에서 실행한다. `BevInput.msg`가 확장되었으므로 카메라와 BEV를
같이 빌드해야 한다. 이번 변경 작업에서는 사용자 요청에 따라 빌드·테스트를 실행하지 않았다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup manual_obstacle_view.launch.py \
  params_file:="$(pwd)/src/vehicle_bringup/config/manual_obstacle_view.yaml"
```

기본 실행은 조이스틱 수동조작, VESC 연결, 차량 운동 모니터, RGB+Depth 카메라,
BEV와 차선인식 프리뷰를 함께 시작한다. `auto_control`과 `roi_lidar` 노드는 실행하지 않는다.
평지에서 정지한 상태로 시작 측정을 끝낸 뒤 프리뷰가 갱신되는 것을 확인하고 수동조작한다.
기존에 수동주행/운동 모니터를 실행했다면 해당 구성만 제외할 수 있다.

```bash
# manual_drive.launch.py가 이미 실행 중인 경우 (운동 모니터는 새로 실행)
ros2 launch vehicle_bringup manual_obstacle_view.launch.py \
  manual_enabled:=false \
  params_file:="$(pwd)/src/vehicle_bringup/config/manual_obstacle_view.yaml"

# manual_drive_with_dynamics.launch.py가 이미 실행 중이면 추가: dynamics_enabled:=false
```

차량 namespace 기본값은 `autopilot03`이다. 필요하면 `vehicle_namespace`, `vesc_port`,
`controller_name_contains`를 지정한다. GUI 없이 실행하려면 `gui:=false`로 두고
`/line_detactor/result_image`를 구독한다. 같은 OAK를 여는 기존 카메라/roi_lidar 실행은 종료한다.

## 파일을 수정하며 조정

`config/manual_obstacle_view.yaml` 한 파일에 세 노드의 실험 설정이 있다.
`params_file`에 소스 파일의 절대 경로를 전달하면 YAML 변경 후 재빌드 없이 launch를 재시작한다.
장애물 옵션은 시작 시 고정하며 `ros2 param set`으로 실행 중 바꾸는 것은 허용하지 않는다.

| 섹션 | 담당 설정 |
|---|---|
| `camera_driver` | Depth 해상도/FPS, IR 강도, 깊이 신뢰도·LR/subpixel |
| `bev_processor` | Depth ROI, 샘플 간격, 바닥 제거, 군집 조건·동기화 |
| `line_detactor` | 장애물 표시와 영상/Depth 시각 차이·만료 제한 |

기존 전체 설정 파일을 바꾸려면 `camera_params_file`, `bev_params_file`, `lane_params_file`로
지정한다. 각 파일에 실험 YAML을 덮어쓴 뒤, 통합 실행에 필요한 입력 연결/표시 모드를 적용한다.
기존 BEV 범위나 모델 크기가 다르면 조용히 축척을 바꾸지 않고 실행 오류로 알린다.

작은 장애물 표본이 부족하면 `obstacles.points.pixel_stride: 1`을 먼저 시도하고,
`obstacles.cluster.min_bins: 1`로 작은 군집도 허용할 수 있다. 전자는 호스트 비용이 늘고,
후자는 고립 노이즈도 표시할 수 있다. `obstacles.scan.min_samples: 1`은 표본 지지를 더 완화한다.

지면 제거 두께는 `min(max_distance_m, distance_m + distance_per_meter × 수평거리)`다.
기본 두께는 1m에서 6cm, 3m에서 8cm이며 군집에 참여하려면 선택 거리 주변에서 10cm 이상
높이 근거가 있어야 한다. 낮은 물체/성긴 반환점이 제외될 수 있다. 이 값은 바닥 완전 제거를
보장하지 않으며, 주행 중 노면 변화·차체 움직임·Depth 오차의 영향을 받는다.

장애물 표시만 끄려면 `line_detactor.obstacles.overlay_enabled: false`로 설정한다.
Depth 부하까지 제거하려면 `camera_driver.obstacles.depth.enabled`와
`bev_processor.obstacles.enabled`도 모두 false로 설정하고 재시작한다.
점 표시에 시간을 누적하거나 추적 ID를 유지하지 않는다. 색상과 #번호는 프레임 내 구분용이다.

## 데이터 흐름과 지연

- `bev_processor`가 한 번 측정한 OAK ID/높이/자세를 사용한다. 이후 `camera_driver`가 그 ID를
  열어 CAM_A RGB와 CAM_B/C StereoDepth를 하나의 DepthAI 파이프라인에서 취득한다.
- Depth는 RAW16 mm + 실제 내부/외부 보정 + 촬영 시각을 유지한다. 호스트 최신 프레임 하나를
  같은 프로세스에서 소유권과 함께 전달하며, 3D 포인트 클라우드를 ROS로 직렬화하지 않는다.
- `bev_processor`의 별도 스레드가 RGB에 실제 사용한 고정 줌/자세 보정과 BEV LUT의 카메라
  모델을 조합해 Depth를 같은 좌표로 변환하고 바닥 제거 → 가상 스캔 → 군집 검출을 수행한다.
  RGB 보정과 Depth는 기본 40ms 이내 촬영 시각으로 연결한다. 보정 행렬 선택은 가까운 RGB
  프레임을 기준으로 하며 Depth 촬영 시각으로 자세를 별도 보간하거나 차량 이동을 보상하지 않는다.
- 차선 BEV/GPU 추론은 장애물 검출 완료를 기다리지 않는다. 차선 GUI에서 기본 60ms 이내의
  군집 결과를 찾아 합성하고, 촬영 후 250ms가 지난 결과는 삭제한다. 입력이 멈춰도 GUI는
  만료 여부를 다시 평가한다. 만료/동기 실패는 `depth WAIT / stale or unsynced`로 표시한다.
- 결과 화면에만 선·대표점·최근접 거리를 그린다. 차선 모델 입력, 라벨, 중앙 경로와 조향/속도
  명령에는 장애물 표시를 반영하지 않는다. 외곽선은 관측된 표면점의 볼록 외곽선이며 실제
  물체 전체 점유영역을 뜻하지 않는다. RGB 지면 투영에서 늘어진 물체 실루엣과는 다를 수 있다.
- 기본 Depth는 400p/60FPS 요청, RGB는 기존 80FPS 요청이다. 실제 속도는 장치/USB/모드와
  GPU·CPU 동시 부하에 따라 달라진다. 이 변경으로 지연이 없어진다고 보장하지 않는다.

화면 하단: `depth ... FPS n=... dt=...ms` (표시 범위 내 군집 수/차선 영상과 시간 차이).
`OBSTACLES` 로그: 실제 처리 FPS, 호스트 ms, 프레임 나이, RGB 보정과 시간 차이,
바닥 제외 수, 높이·크기 조건 탈락 수.
`/bev_processor/obstacles`: 관측 군집의 평균 XY를 PoseArray로 발행한다. Z=0이며 방향은
추정하지 않는다. 차선 합성에는 대표점만이 아니라 내부적으로 전달하는 군집 표면점을 사용한다.

통합 장애물 기능은 기본 BEV/카메라 실행에서 OFF다. 독립 `roi_lidar` 패키지와 설정은 유지한다.
