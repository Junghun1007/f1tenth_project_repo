# ROI terrain + 차선 VFH+ 국소 주행

**기본 실행은 실차 주행 모드**다. Start driving을 누르면 VESC Duty·브레이크·서보 명령을 발행한다.
`drive_enabled:=false`로 실행하면 기존처럼 프리뷰만 출력한다.
`roi_lidar`의 terrain, 같은 OAK RGB에서 만든 차선 결과, VESC 실측 속도와 IMU yaw rate를 사용한다.
기본 목표 속도는 0.7m/s이며, 출발 전 GUI에서 첨부 controller YAML 상한(1.8m/s)까지 조절한다. 좁은 통로·곡률·검사된 경로 길이에 따라 낮아진다.

## 실행

차량의 ROS 2 Humble/DepthAI/TensorRT 환경에서 저장소 루트 기준:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_roi_vfh_low_memory.sh
source install/setup.bash
ros2 launch roi_vfh_test preview.launch.py
```

빌드는 새 터미널에서 ROS만 source한 상태로 진행한다. 빌드 전에 이 저장소의
`install/setup.bash`를 source하면 자기 설치물을 underlay로 인식하는 override 경고가 발생할 수 있다.
빌드 스크립트는 패키지를 순차 처리하고 Make/CMake 컴파일도 1개로 제한한다.
`camera_driver`와 `roi_lidar`가 동시에 컴파일될 때 메모리 사용량이 겹치는 것을 줄이며,
Release 최적화는 유지한다. `BUILD_TESTING=OFF`로 테스트 타깃 생성도 제외한다.

`c++: fatal error: Killed signal terminated program cc1plus`는 컴파일러 프로세스가
강제 종료됐다는 뜻이며 메모리 부족이 흔한 원인이다. DepthAI의 GCC ABI `note`와 구분한다.
순차 빌드에서도 반복되면 `sudo dmesg -T | tail -n 80`에서 OOM 기록을 확인하고
메모리를 사용하는 다른 작업을 종료하거나 시스템 swap 설정을 점검한다.
이 스크립트 추가 시 빌드·테스트는 실행하지 않았다.

기존 camera_driver/auto_drive/point_cloud/roi_lidar를 종료하고 OAK는 이 실행 하나에서 사용한다.
차량은 평지에서 정지한 채 시작 지면 측정과 차선 모델 준비를 완료한다.
ROI는 auto_control/BEV와 같은 `oak_startup` 측정과 BEV 설정의 456×228px/아래 80px ROI를 사용한다.
`OAK startup ground-plane ROI` 창의 빨간 사각형 안에 평평한 노면이 보이게 하고,
측정 완료 시 이 창은 자동으로 닫힌다. 수동 높이 모드는 BEV 설정대로 별도 동작한다.
VESC bridge는 이 launch에서 기본으로 함께 실행한다. `vesc_only.launch.py`와 같은
`vehicle_bringup/config/vesc_config.yaml`을 사용하며 기본 포트는 `/dev/ttyTHS1`이다.
수동/별도 자동주행 제어 노드는 실행하지 않으며, 실제 주행은 기존 Start 버튼 이후에 시작한다.
실차 모드에서는 auto_control/actuator_commander 등 다른 Duty·브레이크·서보 발행 노드를 종료한다.
같은 명령 토픽에 다른 publisher가 감지되면 Start를 거부하거나 주행을 정지한다.
기본 ERPM/연결 토픽은 `config/reference/auto_control_test.yaml`의 `measured_erpm_topic`과 `connection_status_topic`을
가져온다(현재 `/vesc/measured_erpm`, `/vesc/connected`). 바퀴 지름·극쌍·기어비·방향·속도 배율과
속도 필터 시간도 같은 controller YAML에서 가져온다. `vehicle_namespace:=autopilot03`을 붙이면
통합 bridge와 제어·실측속도 노드 모두 같은 namespace의 VESC 토픽을 사용한다. 수동 제어 노드와는 동시 구동하지 않는다.
`measured_erpm_topic:=... vesc_connection_topic:=...`는 namespace보다 우선하는 직접 지정이다.

```bash
# VESC bridge까지 함께 실행 (별도 vesc_only 실행 불필요)
ros2 launch roi_vfh_test preview.launch.py
# 프리뷰만 볼 경우
ros2 launch roi_vfh_test preview.launch.py drive_enabled:=false
```

기존 `vesc_only` 등 별도 bridge를 종료한 뒤 실행한다. 외부 bridge를 계속 사용할 경우에만
`vesc_bridge_enabled:=false`를 지정한다. `drive_enabled:=false` 프리뷰에서도 실측속도 입력을 위해
통합 bridge는 기본으로 실행한다.
포트는 `vesc_port:=/dev/ttyUSB0`, 설정 파일은 `vesc_params_file:=/절대경로/vesc_config.yaml`로 변경한다.
bridge의 Duty·브레이크·서보·ERPM·연결 토픽은 VFH 및 dynamics와 같은 값으로 연결한다.
namespace 및 `duty_topic`, `brake_current_topic`, `servo_position_topic` 직접 지정도 지원한다.
IMU는 별도 camera_driver를 켜지 않고 `/roi_lidar/imu`를 사용한다. 시작 보정으로 이미 차량 좌표로
회전한 각속도이므로 Z축이 yaw이며, 원본 `/camera/imu`의 -Y축 설정을 복사하면 안 된다.
실측 속도 미수신과 IMU 미수신은 별도 정지 사유로 표시한다. IMU는 수신 없음·수신 중단·측정 시각 만료·이동 이력 오류를 구분한다. 속도가 없어도 유효 IMU 수신 시간은
갱신하지만, 속도·IMU가 모두 유효해지기 전에는 촬영 시각 보정이나 계획을 진행하지 않는다.
receive-only dynamics monitor가 이미 있으면 `dynamics_enabled:=false measured_speed_topic:=/실측속도토픽`으로 재사용한다.

BEV는 `ROI VFH+ test` 창 하나로 표시한다. 별도 `ROI LiDAR - RGB BEV` 창은 열지 않으며,
뎁스 ROI 선택창과 ROI 파라미터 창은 함께 유지한다. 일반 roi_lidar 단독 실행은 기존 BEV창을 유지한다.
`preview.show_bev_window`는 ROI BEV창 표시만 제어하며 센서·차선 BEV 발행에는 영향을 주지 않는다.
GTK HighGUI의 `WND_PROP_VISIBLE=-1`은 미지원 값이므로 창 닫힘으로 판단하지 않는다.
GTK에서는 `WND_PROP_AUTOSIZE`를 보조 조회하며, 지원되는 속성을 관측한 뒤에만
창 소멸을 판단한다.

1. 출발 전에는 선택 경로 없음, 목표 속도/조향 0. ROI 창에서 값을 조절하고 Apply한다.
2. **Start driving**: ROI 파라미터 잠금 ACK, VESC 연결, 최신 입력을 확인한 뒤 계획과 실제 구동을 시작한다.
   프리뷰 모드는 버튼이 **Start preview**로 표시되며 액추에이터 publisher를 만들지 않는다.
3. 실행 중에는 ROI GUI 입력 및 `ros2 param set /roi_lidar ...`를 거부한다.
4. **Stop / unlock**, Space 또는 S: 구동 Duty 0, 기존 `brake_maximum_current_amps`(현재 2A) 제동,
   마지막 서보 조향 유지. 진단 경로/목표 속도·조향은 0으로 초기화하고 ROI 잠금을 해제한다.
5. Q/Esc/창 닫기도 Stop 처리한다. Stop/종료, VESC 연결·ROI 잠금 유실, 중복 구동 publisher,
   좌표/메시지 오류는 자동 재출발하지 않는다.
6. 차선·terrain의 촬영 후 허용 시간은 **0.30초**다. 새 동기 입력을 기다리는 동안에는
   기존 계획에 사용한 차선과 terrain이 모두 이 시간 안에 있을 때 직전 경로를 유지한다.
   수명이 지난 입력의 timestamp를 새로 찍어 유효한 것처럼 연장하지 않는다.
7. 입력 지연·유실(차선/terrain/속도/IMU/촬영 시각 정렬) 때는 `WAIT` 상태로 제동한다.
   Start 요청과 ROI/VFH 파라미터 잠금은 유지하며, 최신 입력과 유효 경로가
   **0.50초 연속 정상**이면 자동 재출발한다. 기다리는 중 Stop을 누르면 자동 재출발을 취소하고 잠금을 푼다.
   유효 입력에서 경로만 막힌 경우에도 제동하면서 재계획한다.

VFH 고유 설정은 `config/preview.yaml`, 대응하는 차량·제어·센서·인식 설정은 `config/reference`의 첨부 YAML에서 가져온다.
우선순위와 적용 범위는 [APPLIED_SETTINGS.md](config/reference/APPLIED_SETTINGS.md)를 참고한다.
VFH 실행창 상단 **VFH parameters** 또는 **P**로 같은 크기의 설정 화면을 연다.
**Prev/Next** 또는 **[ / ]**로 두 페이지를 전환한다. Apply와 Start는 모든 페이지의 입력을 함께 적용한다.
값을 클릭해 숫자를 입력하고 **Apply / Enter / A**로 일괄 적용한다. Tab은 다음 항목,
입력 중 Esc는 해당 필드 취소다. **Reload**는 현재 적용값을 다시 읽어 미적용 입력을 버린다.
Start를 누르면 미적용 입력도 검증·적용한 뒤 출발한다. 잘못된 값이면 출발하지 않고 이유를 표시한다.
Start를 요청한 순간(ROI 잠금 ACK 대기 포함)부터 VFH GUI와 `ros2 param set`을 잠그며,
Stop을 눌러 자동 재출발을 취소하면 다시 편집할 수 있다. 입력 대기 중에는 잠금을 유지한다. 잠긴 상태에서도 설정값 조회·화면 전환은 가능하다.
ROI 파라미터 창의 기존 잠금도 함께 유지한다. Apply는 카메라를 재시작하지 않는다.

| 실행창 항목 | 기본값 | 의미 |
|---|---:|---|
| `nominal_speed_mps` | 0.7 | 최대 목표 속도, 0.05~첨부 YAML 상한(현재 1.8)m/s |
| `lane_lookahead_m` | 0.05 | Stanley 접선 방향 계산 거리, 0.01~0.8m |
| `safety_margin_m` | 0.04 | 첨부 obstacles 기준 좌우/앞 여유, 0~0.2m; 불확실성 0.01m 별도 |
| `horizon_m` | 1.4 | 검사할 궤적 길이, 0.3~2.5m |
| `reference_distance_m` | 1.0 | VFH 후보 목표 거리, 0.3~3m |
| `sample_step_m` | 0.02 | 궤적 검사 간격, 0.02~0.08m |
| `lane_departure_m` | 0.40 | 일반 회피의 차체 차선 이탈 한도, 0~0.4m |
| `deceleration_mps2` | 0.5 | 계획 감속도; 시작 시 controller에서 가져온 상한 이하 |
| `response_delay_sec` | 0.20 | 계획 반응 시간, 0~1초; 프레임 처리 지연 하한은 유지 |
| `histogram_low` | 1.0 | VFH 낮은 임계값; high보다 작아야 함 |
| `histogram_high` | 2.0 | VFH 높은 임계값; low보다 커야 함 |

GUI와 ROS 변경은 같은 검증을 사용하며 서로 반영된다. 적용 시 이전 계획·히스토그램을 초기화한다.
차량 형상·조향 한계·구동 PID·토픽 등은 기존처럼 시작 시 고정한다.
실행 중 수정은 이번 세션에 적용된다. 재실행 기본값은 대응하는 첨부 YAML이 우선하며, VFH 고유 값은 `config/preview.yaml`에서 정한다.
이번 파라미터 GUI 변경은 사용자 요청대로 **빌드·테스트를 실행하지 않았다.**
비정상 프로세스 강제 종료 후 ROI만 남아 잠겨 있으면 테스트 launch를 다시 실행하거나 다음으로 해제한다:

```bash
ros2 topic pub --once /roi_vfh_test/running std_msgs/msg/Bool '{data: false}'
```

## 알고리즘과 미션

[VFH+ 원 논문](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHplus.pdf)의
4단계(주 극좌표 히스토그램 → 이중 임계값 이진 히스토그램 → 최소 회전 원에 따른 마스크 → 후보 방향 비용)를 사용한다.
이 테스트의 차이는 전방 ±75°/2.5° 섹터, 셀 면적 정규화, 미관측 셀의 통과 허용,
차폭 기반 각도 확장 뒤 직사각형 차체 추가 검사, 빈 방향 안의 추가 후보와 3가지 조향 프로파일이다.
원 논문의 원형 로봇/360°/누적 certainty 구현과 수치적으로 동일한 재현은 아니다.

- 일반 구간: **차선 궤적 우선**. 인식된 중앙선 전체를 따라 매 step 전방 목표를 갱신하는
  auto_control의 Stanley 제어로 궤적을 먼저 생성한다(`lane_lookahead_m=0.05`). 차량 조향 한계·변화율과 차체를 검사하고,
  충돌이 없으면 이 궤적을 그대로 선택하며 VFH 히스토그램을 건너뛴다. 옆쪽 장애물 때문에 우회하지 않는다.
- 소량 콘: 차선 추종 차체 궤적이 **현재 빨간 셀과 충돌할 때만** VFH+를 사용한다.
  전진 길이·방향 안정성과 함께 전체 차선에서의 평균 이탈, 끝점 이탈·접선 방향을 평가한다.
  장애물을 통과한 뒤 차선 추종 궤적이 다시 열리면 일반 차선 추종으로 복귀한다.
  관측된 장애물, 차체 외곽의 차선 밖 각각 40cm, 회전 한계를 검사한다. 미관측 셀은 통과 가능으로 취급한다.
- 유도 경로: 현재 프레임에서 양쪽 위험 경계, 장애물 없이 연결 가능한 입구, 0.35m 이상 종방향 근거를
  3개의 새 관측에서 확인한다. 원래 차선 궤적의 차체 충돌도 있어야 유도 모드에 진입한다.
  확인 뒤에는 통로 중심 곡선을 추종하고 40cm 제한을 해제한다. 통로 기준 궤적도 막히면 VFH+로 후보를 찾는다.
  확인된 통로 바깥을 가로지르는 후보는 제외한다. 한쪽 경계만으로 유도 모드에 진입하지 않는다.
- 유도 구간에서 차선 중심선이 사라져도 유효한 LaneResult 메시지와 양쪽 통로 관측이 계속 있으면
  통로 방향으로 계획한다. 일반 구간의 차선 유실과 차선 노드 자체의 입력 중단은 정지한다.
- 완전히 막히면 속도 0으로 재계획한다. 탐색 범위 밖의 우회 경로 존재를 보장하지 않는다.

terrain은 첨부 ROI YAML대로 폭 **1.5m × 전방 4m**다. 허용된 40cm 중 이 격자 범위 안의 부분만 쓸 수 있다.
차선 모델은 깨끗한 **120×300 / 1cm** BEV를 받으며 HUD/점/terrain 색상은 입력하지 않는다.
범위 밖, 현재 위험 근거가 있는 셀은 계속 통과 불가다.

사용자 요청에 따라 **격자 안의 미관측(-1)은 주행 가능한 것으로 가정**한다.
히스토그램·회전 마스크·차체 검사·통로 내부 연결 검사에서 미관측을 장애물로 세지 않는다.
양쪽 유도 경계 확인에는 현재 관측된 장애물 근거가 여전히 필요하다.
원본 terrain에 가짜 바닥을 기록하지 않으며, 화면에서도 미관측은 회색이다.
색상과 판정은 동일한 현재 격자를 사용한다. 연속성 필터 ON이면 미확정 작은 hazard를 회색으로 바꾼 격자를 함께 사용한다. 민트색(0)·회색(<50)은 허용하고 빨간색(>=50)만 차단한다.
따라서 표시되는 `checked length`는 미관측 통과 가정을 포함한 검사 길이이고 관측된 평지만의 길이가 아니다.
속도/IMU/차선 입력 조건은 유지한다.

## 현재 프레임과 촬영 시각 보정

**Short-term terrain memory는 제거했다.** 이전 프레임의 바닥·장애물을 누적하거나
현재 미관측 셀에 재투영하지 않는다. `memory_time_sec`, `memory_distance_m`, 청록색 기억 표시는 삭제했다.
계획과 프리뷰는 같은 현재 `terrain_for_planning`에 VFH hazard 연속성 필터를 적용한 격자를 사용한다.
이전 프레임에만 존재하는 장애물을 현재 지도에 복원하지 않는다.

촬영 시각이 다른 차선과 depth를 정렬하기 위한 `MotionHistory`는 유지한다.
같은 OAK 세션에서 차량 좌표로 보정한 gyro Z와 VESC 속도를 적분하고,
앞차축의 횡방향 이동(축간거리 × yaw rate)을 반영해 RGB 차선을 depth 촬영 시각으로 변환한다.
이 이력은 terrain 셀을 저장하지 않는다. IMU 0.10초/속도 0.15초 유실 시 일시 제동하며, 정상 입력·유효 경로 0.50초 유지 후 자동 재출발한다.
VFH 이중 임계값·직전 방향과 유도 모드 연속 확인은 결정의 흔들림 방지용으로 유지한다.
차선이 열려 기본 추종으로 복귀하면 이전 히스토그램 상태는 초기화한다.
움직이는 물체의 의미 추적이나 차량 추종은 이 패키지 범위에 없다.

차량 폭은 `obstacles.avoidance.vehicle_half_width_m × 2`(0.30m), 앞차축부터 앞범퍼는
`traffic_front_axle_to_bumper_m`(0.05m), 뒤쪽 검사 길이는 `vehicle_half_length_m`(0.25m)이다.
이 값은 첨부 설정의 검사 모델이며 전체 차체 실측 형상을 보증하지 않는다. 축간거리 0.33m, 최대 조향 30°, 조향 변화율 240°/s는 기존 제어 설정을 따른다.

사용자 선택대로 **전방 관측 영역만 검사**한다. 음의 X 후방의 차체 회전 공간은 검증하지 않으며
화면·상태에 `rear unverified`로 표시한다. 격자 안의 미관측 통과 허용과 후방 미검증은 실차 모드에도 그대로 적용된다.

## 차선 기준선과 회피 경로

파란 선은 인식한 **차선 기준선**이고, 장애물이 겹쳐도 그대로 표시한다.
실제 선택된 주행 경로(차선 추종 또는 VFH+ 회피)는 **노란 선**이며 Start/잠금 완료 후 유효 경로가 있을 때만 표시한다.
회색 선은 검사한 후보 궤적이다. 파란 선이 장애물을 가로지른다고 그 선으로 구동하는 것은 아니다.
통과 가능한 VFH 방향이 없으면 `no free VFH+ direction`, 후보의 차체 검사가 모두 짧게 끝나면
`no path >=0.12m`와 종료 원인별 횟수를 표시한다.
`Ends: hazard / edge`는 후보가 장애물/격자 끝에서 중단된 횟수,
`Route limit / short`는 차선·유도 통로 제한으로 중단된 횟수/0.12m 미만 후보 수다.
차선 추종 후보 1개와 VFH 후보 최대 51개를 검사한다. 유효 후보도 먼 지점에서 중단될 수 있으므로 종료 횟수와 탈락 후보 수는 같은 의미가 아니다.
`Longest candidate`로 확보한 최대 검사 길이를 확인한다. 단순 빈 공간 유무 외에도 차폭·셀 여유,
회전반경·앞 돌출·격자 폭·차선 이탈 제한을 만족해야 회피할 수 있다.
이번 첨부 YAML 반영은 사용자 요청대로 빌드·테스트를 실행하지 않았다.

## 실차 연결과 선택한 여유

- `safety_margin_m=0.04`: 첨부 obstacles 설정대로 좌우/앞 각각 4cm와 불확실성 1cm를 적용한다.
  폭 30cm 기준 검사 폭은 40cm다. 회전한 차체 직사각형과 셀 사각형의 교차를 직접 검사하며,
  셀 대각선 길이를 고정 패딩으로 추가하지 않는다. GUI에서 margin을 변경할 수 있다.
- `nominal_speed_mps=0.7`: 상한이며, 곡률·남은 검사 길이에 따라 감속한다.
- 정지 시 구동 Duty 0 + 기존 최대 브레이크 전류, 조향 유지. 출발 전에도 제동하며,
  최초 Start 전에는 서보를 임의로 중앙 정렬하지 않는다.

`auto_control::control_core`의 기존 `SpeedPid`, `staged_brake_current`, `move_toward`,
`steering_angle_to_servo`를 공유한다. 전체 auto_control 노드를 켜지 않으므로 기존 1m/s 속도 하한이나
별도 차선 추종 결과가 VFH 목표를 덮지 않는다. PID/FF·최대 Duty·출발 Duty·slew·브레이크·서보 반전은
controller YAML에서 `drive.*` 파라미터로 전달한다. 이 어댑터는 VFH의 속도 오차에 맞춰 가감속하며,
auto_control의 신호등/정지선/코너 상태기를 복제하지 않는다.
제동 중에는 모터 모드를 브레이크 하나로 보내고, 0A 해제를 별도 주기에 발행한 뒤 Duty를 재개한다.
제동 중 `command_duty=0`이며, 실제 `/vesc/duty`에 0을 동시 발행해 브레이크를 덮어쓰지 않는다.
최대 계획 입력 수명은 사용자 선택대로 VFH `max_age_sec=0.30`을 사용하며,
구동 감시의 `drive.max_command_age_sec`도 같은 값을 따른다. 차선 수신 timeout은
controller 설정과 이 수명 중 큰 값(현재 0.30초)을 사용한다. 재출발 대기 시간은
`input_resume_stable_sec=0.50`이다. 이 값들은 실행 YAML에서 지정하는 시작 시 고정 파라미터다.
계획에 사용한 차선과 terrain 촬영 수명을 각각 검사한다.
계획 감속도는 controller의 `longitudinal_planning_deceleration_mps2` 이하(현재 0.5m/s²)로 제한한다.
강제 종료·통신 단절 때의 최종 모터 명령 만료는 기존 bridge watchdog을 따른다.

계획은 20Hz, 구동·정지 감시는 100Hz다. 첨부 설정대로 `driving_logs`에 최대 30Hz CSV를
비동기로 기록하며, 적용된 파라미터·상태·속도·Duty·브레이크·조향·경로를 함께 남긴다.

## 출력과 검증

`/roi_vfh_test/path`, `/roi_vfh_test/target_speed`, `/roi_vfh_test/steering_angle_rad`,
`/roi_vfh_test/status`, `/roi_vfh_test/preview`는 진단 출력이다.
실차 모드에서는 `/vesc/duty`, `/vesc/brake_current`, `/vesc/servo_position`에 실제 명령을 발행한다
(namespace/직접 지정 적용). `/roi_vfh_test/command_duty`, `/roi_vfh_test/command_brake_current`,
`/roi_vfh_test/command_servo_position`은 최종 출력 진단이다.
`/roi_lidar/lane_bev`, `/roi_lidar/imu`, `/roi_lidar/terrain_for_planning`은 이 launch에서 센서 입력으로 사용한다.
일반 ROI 실행은 새 센서 스트림이 기본 OFF이므로 기존 동작을 유지한다.

```bash
colcon test --packages-select roi_lidar roi_vfh_test --event-handlers console_direct+
colcon test-result --verbose
```

기존 테스트 코드는 순수 C++ 계획/촬영 시각 보정, 프리뷰 렌더와 버튼 영역, launch 구성,
ROI 격자/스캔/프리뷰/설정 및 GUI 잠금을 검사한다. 이번 변경에서는 위 명령과 개별 테스트, 빌드를 모두 실행하지 않았다.
ROS 전체 실행, 실제 IMU 축/속도 환산, 모델 추론 지연과 버튼-파라미터 ACK 연결은 차량 환경에서 확인한다.

## 로컬 연산 측정

재현 명령(ROS 불필요):

```bash
c++ -std=c++17 -O2 -Isrc/roi_vfh_test/include -Isrc/auto_control/include \
  src/roi_vfh_test/test/benchmark.cpp src/auto_control/src/control_core.cpp -o /tmp/roi-vfh-benchmark
/tmp/roi-vfh-benchmark
```

1440셀의 정지 장면 4개에서 20회 워밍업 후 300회 측정한다. 현재 격자의 계획만 측정하며
카메라/차선 추론/ROS/GUI 비용은 제외한다. 전체 Jetson 실행시간은 해당 기기에서 측정해야 한다.
노드 HUD의 `plan` 시간은 계획부만 측정한다.

## IMU 수신과 최신 ROI 기본값

`auto_control_node` 자체는 IMU를 직접 구독하지 않는다. auto_control 실행 구성의
`camera_driver`는 전용 IMU 스레드를 사용한다. ROI도 동일하게 depth 처리와 별개인
IMU 스레드로 큐를 지속 수신한다. RANSAC/고립점 처리에 IMU 수신이 묶이지 않도록 했다.
보정 gyro에서 카메라 좌표로의 회전은 camera_driver와 같은 방식이며,
ROI는 시작 지면 측정 회전을 추가 적용하므로 yaw는 차량 Z축이다.
별도 camera_driver를 추가 실행할 필요가 없다.

VFH의 IMU 수신 큐는 영상의 latest-only 큐와 분리해 200개로 제한한다.
중복·역순 timestamp와 오래된 패킷은 이력을 지우지 않고 무시한다.
실제 측정 시각이 0.10초를 넘으면 계속 제동하며, 수신 시각으로 덮어 만료를 숨기지 않는다.
기존 정상 입력 0.50초 후 자동 재출발 정책을 유지한다.

2026-11-06 16:32 첨부 이미지 기본값을 ROI 일반 설정과 VFH ROI 프로필에 반영했다.
주요 변경은 terrain 셀 0.07m/장애물 표본 8개, RANSAC 허용 거리 0.06m/표본 700개,
최소 장애물 높이 0.06m, 고립점 반경 0.08m/이웃 8개,
고정 지면 두께 0.06m+거리당 0.01m, 높이 상한 1m, depth 60FPS, terrain 1.5×4m다.
이번 수정은 빌드·테스트를 실행하지 않았다.

VFH 기본 차선 프로필 `config/reference/line_detactor_test.yaml`의
`centerline_corner_outward_offset_m`은 사용자 요청대로 0.0m다.
코너 외측 추가 오프셋을 끄며 기존 외측 경계 형상 가중치 설정은 유지한다.
VESC 통합/오프셋 변경 시 빌드·테스트는 실행하지 않았다.

## 작은 hazard 연속성 확인

VFH 입력에만 적용한다. ROI의 원본 terrain 발행·평면 추정은 변경하지 않는다.
현재 hazard(>=50)를 8방향 연결 성분으로 나누어, 기본 1~5셀 덩어리의 각 셀은
차량 이동·회전을 보정한 위치에서 2개 depth 관측 연속으로 확인되어야 빨간색으로 반영한다.
6셀 이상 덩어리는 즉시 반영한다. 따라서 큰 덩어리 형태의 클러터까지 제거하는 필터는 아니다.
최소 depth 표본 수 8개와 이 연결 셀 수 기준은 별개다.

| VFH 설정 항목 | 기본값 | 의미 |
|---|---:|---|
| `hazard_filter_enabled` | 1 | 0=OFF, 1=ON. 1페이지 마지막 항목 |
| `hazard_confirm_frames` | 2 | 필요한 연속 관측 횟수, 정수 1~5 |
| `hazard_small_max_cells` | 5 | 확인 대상으로 둘 작은 덩어리 최대 셀 수, 정수 1~50. 이보다 크면 즉시 반영 |
| `hazard_match_radius_cells` | 1 | 이동 보정 후 대응 검색 반경, 정수 0~2. 1은 주변 3×3셀, 2는 5×5셀 |
| `hazard_max_gap_sec` | 0.15 | 연속 관측 사이 최대 시간, 0.02~0.5초. 넘으면 횟수를 다시 시작 |

확인 전 작은 hazard는 회색이며 현재 정책에 따라 통과 가능하다. 초록색으로 만들거나
빈 셀 채우기 대상으로 사용하지 않는다. 작은 실제 장애물도 확인 횟수만큼 반영이 늦어질 수 있다.
현재 관측에서 사라지면 이전 빨간 셀을 남기지 않는다. 한번 크게 관측되어 즉시 인정된
장애물이 다음 프레임에 작게 분할돼도 위치가 대응되면 확인된 상태를 이어간다.

VESC 속도와 차량 좌표 IMU yaw로 이전 검출 위치를 현재 촬영 좌표로 변환한다.
이동 이력이 없으면 작은 hazard를 새로 확정하지 않으며 기존 입력 유효성 검사도 유지한다.
상수 크기 이웃 검색(최대 5×5)과 격자 연결 성분 검사만 사용한다.

수신한 depth 격자는 최대 16개까지 보관하고, 선택한 동기 프레임까지 시간순으로 검사한다.
중간의 수신 프레임에서 사라졌다면 연속 횟수가 끊긴다. 수신하지 못한 프레임은 검사할 수 없다.
최근 16개 프레임의 처리 결과를 timestamp별로 재사용해 같은 depth로 차선만 재계획해도
확인 횟수를 올리지 않는다. 이 캐시는 재처리 방지용이며 과거 지도를 누적하는 기능이 아니다.

Stop/Start, 입력 일시 정지 진입, 설정 변경, 이동 이력 재설정, 격자 형상 변경 시
확인 이력을 초기화한다. 각 프레임의 시간 간격 제한도 적용한다.
출발 후에는 ROI/VFH 편집 잠금을 그대로 유지하며, 페이지 이동과 조회만 가능하다.
GUI에는 원본/확정/미확정/즉시 반영 hazard 셀 수를 표시한다.
`~/status` 메시지와 주행 CSV의 전용 열에도 같은 통계와 필터 처리 시간을 남긴다.
설정은 기존 GUI와 같은 double 숫자형 ROS 파라미터다(예: `hazard_confirm_frames: 2.0`).
이번 변경에서는 빌드·테스트를 실행하지 않았다.
