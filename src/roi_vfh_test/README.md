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
VESC bridge는 별도 실행한다. 이 launch는 bridge나 수동/자동주행 제어 노드를 시작하지 않는다.
실차 모드에서는 auto_control/actuator_commander 등 다른 Duty·브레이크·서보 발행 노드를 종료한다.
같은 명령 토픽에 다른 publisher가 감지되면 Start를 거부하거나 주행을 정지한다.
기본 ERPM/연결 토픽은 `config/reference/auto_control_test.yaml`의 `measured_erpm_topic`과 `connection_status_topic`을
가져온다(현재 `/vesc/measured_erpm`, `/vesc/connected`). 바퀴 지름·극쌍·기어비·방향·속도 배율과
속도 필터 시간도 같은 controller YAML에서 가져온다. bridge가 `/autopilot03` namespace로
실행 중이면 `vehicle_namespace:=autopilot03`을 붙인다. 수동 제어 노드와는 동시 구동하지 않는다.
`measured_erpm_topic:=... vesc_connection_topic:=...`는 namespace보다 우선하는 직접 지정이다.

```bash
# vesc_only.launch.py로 root /vesc 텔레메트리를 이미 실행한 경우
ros2 launch roi_vfh_test preview.launch.py
# 프리뷰만 볼 경우
ros2 launch roi_vfh_test preview.launch.py drive_enabled:=false
```

VESC bridge가 아예 없다면 별도 터미널에서 `ros2 launch vehicle_bringup vesc_only.launch.py`로
root bridge를 실행할 수 있다. 기존 bridge와 중복 실행하지 않는다.
namespace 및 `duty_topic`, `brake_current_topic`, `servo_position_topic` 직접 지정도 지원한다.
IMU는 별도 camera_driver를 켜지 않고 `/roi_lidar/imu`를 사용한다. 시작 보정으로 이미 차량 좌표로
회전한 각속도이므로 Z축이 yaw이며, 원본 `/camera/imu`의 -Y축 설정을 복사하면 안 된다.
실측 속도 미수신과 IMU 미수신은 별도 정지 사유로 표시한다. 속도가 없어도 유효 IMU 수신 시간은
갱신하지만, 속도·IMU가 모두 유효해지기 전에는 촬영 시각 보정이나 계획을 진행하지 않는다.
receive-only dynamics monitor가 이미 있으면 `dynamics_enabled:=false measured_speed_topic:=/실측속도토픽`으로 재사용한다.

ROI RGB BEV/파라미터 창과 `ROI VFH+ test` 창이 함께 열린다.
GTK HighGUI의 `WND_PROP_VISIBLE=-1`은 미지원 값이므로 창 닫힘으로 판단하지 않는다.
GTK에서는 `WND_PROP_AUTOSIZE`를 보조 조회하며, 지원되는 속성을 관측한 뒤에만
창 소멸을 판단한다.

1. 출발 전에는 선택 경로 없음, 목표 속도/조향 0. ROI 창에서 값을 조절하고 Apply한다.
2. **Start driving**: ROI 파라미터 잠금 ACK, VESC 연결, 최신 입력을 확인한 뒤 계획과 실제 구동을 시작한다.
   프리뷰 모드는 버튼이 **Start preview**로 표시되며 액추에이터 publisher를 만들지 않는다.
3. 실행 중에는 ROI GUI 입력 및 `ros2 param set /roi_lidar ...`를 거부한다.
4. **Stop / unlock**, Space 또는 S: 구동 Duty 0, 기존 `brake_maximum_current_amps`(현재 2A) 제동,
   마지막 서보 조향 유지. 진단 경로/목표 속도·조향은 0으로 초기화하고 ROI 잠금을 해제한다.
5. Q/Esc/창 닫기도 Stop 처리한다. 센서·차선·속도·IMU·연결·잠금 유실은 정지하고 다시 Start를 요구한다.
   유효 입력에서 경로만 막힌 경우에는 제동하면서 재계획한다.

VFH 고유 설정은 `config/preview.yaml`, 대응하는 차량·제어·센서·인식 설정은 `config/reference`의 첨부 YAML에서 가져온다.
우선순위와 적용 범위는 [APPLIED_SETTINGS.md](config/reference/APPLIED_SETTINGS.md)를 참고한다.
VFH 실행창 상단 **VFH parameters** 또는 **P**로 같은 크기의 설정 화면을 연다.
값을 클릭해 숫자를 입력하고 **Apply / Enter / A**로 일괄 적용한다. Tab은 다음 항목,
입력 중 Esc는 해당 필드 취소다. **Reload**는 현재 적용값을 다시 읽어 미적용 입력을 버린다.
Start를 누르면 미적용 입력도 검증·적용한 뒤 출발한다. 잘못된 값이면 출발하지 않고 이유를 표시한다.
Start를 요청한 순간(ROI 잠금 ACK 대기 포함)부터 VFH GUI와 `ros2 param set`을 잠그며,
Stop 또는 입력 유실로 정지하면 다시 편집할 수 있다. 잠긴 상태에서도 설정값 조회·화면 전환은 가능하다.
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
색상과 판정은 동일한 현재 격자를 사용한다. 민트색(0)·회색(<50)은 허용하고 빨간색(>=50)만 차단한다.
따라서 표시되는 `checked length`는 미관측 통과 가정을 포함한 검사 길이이고 관측된 평지만의 길이가 아니다.
속도/IMU/차선 입력 조건은 유지한다.

## 현재 프레임과 촬영 시각 보정

**Short-term terrain memory는 제거했다.** 이전 프레임의 바닥·장애물을 누적하거나
현재 미관측 셀에 재투영하지 않는다. `memory_time_sec`, `memory_distance_m`, 청록색 기억 표시는 삭제했다.
계획과 프리뷰는 같은 현재 `terrain_for_planning`을 사용한다.

촬영 시각이 다른 차선과 depth를 정렬하기 위한 `MotionHistory`는 유지한다.
같은 OAK 세션에서 차량 좌표로 보정한 gyro Z와 VESC 속도를 적분하고,
앞차축의 횡방향 이동(축간거리 × yaw rate)을 반영해 RGB 차선을 depth 촬영 시각으로 변환한다.
이 이력은 terrain 셀을 저장하지 않는다. IMU 0.10초/속도 0.15초 유실 시 정지하며 다시 Start가 필요하다.
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
최대 계획 입력 수명은 controller의 `path_capture_maximum_age_sec`(현재 0.20초),
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
