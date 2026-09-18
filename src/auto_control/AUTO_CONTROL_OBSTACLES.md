# 기존 auto_control 통합 실행에서 장애물 매핑 확인

기존 `vehicle_bringup/auto_drive.launch.py`에 장애물 검출을 연결했다.
**차선 검출 + 신호등 검출 + 장애물 BEV 매핑**을 함께 실행하며,
`manual_test:=true`로 자동 제어 출력을 끄고 조이스틱으로 수동주행할 수 있다.
BEV는 기존 **120cm × 전방 300cm**, 딥러닝 차선 모델 입력은 **120×300px**를 유지한다.

## 이번 수동 실험 실행

차량 PC의 저장소 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=true \
  obstacle_params_file:="$(pwd)/src/auto_control/config/obstacles.yaml"
```

평지에서 정지한 상태로 시작 측정과 모델 준비를 끝낸 후 수동주행한다.
이 실행은 수동조작·VESC·운동 모니터도 함께 켠다. 기존 수동주행이 이미 실행 중이면
`manual_enabled:=false`를 추가한다. 운동 모니터도 실행 중이면 `dynamics_enabled:=false`를 추가한다.
수동 차량 namespace 기본값은 `autopilot03`이며 `vehicle_namespace:=...`로 변경할 수 있다.
기존 카메라/auto_drive 실행은 종료하고 이 명령 하나만 사용한다.

`manual_test:=true`는 기존 `auto_control_node`를 `monitor_only`로 유지한다. 차선 추종 계산과
진단은 계속하고 조향·구동·제동 명령을 발행하지 않으므로 수동조작과 충돌하지 않는다.
외부 auto_control YAML이 drive여도 수동 실험 옵션이 우선한다. 신호등 정지 제어는 OFF이고,
성능 측정용 자동 종료/GUI 비활성화도 해제한다. `auto_enabled=false`만 사용하는 방법은
정지 명령이 수동조작에 섞일 수 있어 이 실험에서는 monitor_only를 사용한다.

## 구성과 표시

- 기존 `camera_driver`가 한 OAK에서 RGB와 스테레오 Depth를 함께 받는다.
- `bev_processor`는 기존 영상 변환과 시작 측정/기하 공유를 담당한다.
- **`auto_control::ObstacleDetectorNode` (`/auto_obstacles`)**가 별도 스레드에서
  ROI 깊이 → 바닥 제거 → 각도별 거리 → 군집 검출을 수행한다. 제어 루프와는 별도로 동작한다.
- 기존 `line_detactor` 딥러닝 모델과 `traffic_light_detector`를 사용한다. 신호등은
  BEV 변환 전 RGB를 공유하고, 장애물은 같은 프로세스의 Depth를 공유한다.
- 기존 차선 결과 프리뷰에 군집 외곽선·대표점·거리, 하단에 Depth FPS·시각 차이와
  `Traffic: RED / GREEN / UNKNOWN`이 표시된다.
- 장애물 위치 토픽은 `/auto/obstacles` (PoseArray)다. XY는 앞차축 기준 관측 표면의 평균,
  Z=0, 방향은 추정하지 않는다. 화면 합성에는 군집 표면점도 내부적으로 전달한다.
- 장애물 외곽선은 관측 표면의 볼록 외곽선이고 물체 전체 영역을 복원한 것은 아니다.
  색상/#번호는 프레임 안에서의 구분이며 추적 ID가 아니다.

아직 장애물로 경로를 바꾸거나 자동 제동하지 않는다. 매핑이 확인된 뒤 회피 경로계획을
추가한다. `manual_test:=false`인 기존 자동주행 모드도 현재는 장애물을 표시만 하므로
장애물 회피 주행이 구현된 것으로 해석하면 안 된다.
독립 `roi_lidar` 패키지는 유지하며 이 통합 실행에서는 시작하지 않는다.

## YAML 조정

`src/auto_control/config/obstacles.yaml`을 직접 수정하고 launch를 재시작한다.
위 명령처럼 소스 YAML 경로를 넘기면 YAML 변경 후 재빌드가 필요 없다.
네 섹션은 모두 `obstacles.*` 옵션만 허용한다.

| YAML 섹션 | 담당 |
|---|---|
| `camera_driver` | Depth 해상도/FPS/IR 강도·신뢰도·LR/subpixel |
| `bev_processor` | `obstacles.share_reference`: 시작 기하 공유 |
| `auto_obstacles` | ROI, 바닥 제거, 스캔·군집 및 동기화 |
| `line_detactor` | 장애물 표시·촬영 시각 차이·만료 |

실행 시 `obstacles_enabled`가 카메라 Depth·기하 공유·검출 노드·화면 합성을 한 번에 켜고 끈다.
기본 true이며 부하 비교는 `obstacles_enabled:=false`로 한다.
차선·신호등·제어의 기존 파일은 각각 `line_detactor_params_file`, `traffic_light_params_file`,
`auto_control_params_file`로 지정한다. 신호등은 기존 `traffic_light_enabled` 옵션을 유지한다.

작은 장애물은 `obstacles.points.pixel_stride: 1`로 표본을 늘리고,
`obstacles.cluster.min_bins: 1`로 작은 묶음을 허용할 수 있다. 연산량/노이즈가 증가할 수 있다.
바닥 제거 두께는 `min(max_distance_m, distance_m + distance_per_meter × 수평거리)`다.
기본 1m에서 6cm, 3m에서 8cm 이하를 제외하고 군집 방향에는 10cm 이상의 높이 근거를 요구한다.
낮은 장애물도 제외될 수 있다. 모든 장애물 옵션은 시작 시 적용하며 YAML 저장 후 재실행한다.

## 시간·성능·검증 범위

시작할 때 한 번 측정한 평면과 RGB에 실제 사용한 고정 줌/자세 보정, 프레임 내부·외부
보정을 사용해 동일 BEV 좌표로 변환한다. Depth와 RGB 보정의 촬영 시각 차이는 기본 40ms,
차선 화면과 장애물은 60ms 이내로 제한한다. 촬영 후 250ms가 지난 군집은 GUI에서 지운다.
`depth WAIT / stale or unsynced`는 신선한 대응 결과를 기다리는 상태다.
차량 이동/Depth 시각으로 자세를 별도 보간하지 않으므로 주행 중 정합은 실측이 필요하다.

카메라/Depth 큐는 최신 한 장을 유지하며 전체 포인트 클라우드의 ROS 직렬화를 하지 않는다.
차선/신호등은 GPU, 군집은 CPU에서 동작하나 USB·메모리·GPU 자원 경쟁은 남는다.
Depth 기본 요청은 400p/60FPS다. 실제 속도는 화면과 `OBSTACLES` 로그에서 확인한다.
코드 수정에서는 요청대로 빌드·테스트·실차 실행을 하지 않았다.
