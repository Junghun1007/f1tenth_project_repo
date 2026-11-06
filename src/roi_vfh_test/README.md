# ROI terrain + 차선 VFH+ 국소 주행

**기본 실행은 실차 주행 모드**다. Start driving을 누르면 VESC Duty·브레이크·서보 명령을 발행한다.
`drive_enabled:=false`로 실행하면 기존처럼 프리뷰만 출력한다.
`roi_lidar`의 terrain, 같은 OAK RGB에서 만든 차선 결과, VESC 실측 속도와 IMU yaw rate를 사용한다.
목표 속도 상한은 0.7m/s다. 좁은 통로·곡률·검사된 경로 길이에 따라 낮아진다.

## 실행

차량의 ROS 2 Humble/DepthAI/TensorRT 환경에서 저장소 루트 기준:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to roi_vfh_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roi_vfh_test preview.launch.py
```

기존 camera_driver/auto_drive/point_cloud/roi_lidar를 종료하고 OAK는 이 실행 하나에서 사용한다.
차량은 평지에서 정지한 채 시작 지면 측정과 차선 모델 준비를 완료한다.
VESC bridge는 별도 실행한다. 이 launch는 bridge나 수동/자동주행 제어 노드를 시작하지 않는다.
실차 모드에서는 auto_control/actuator_commander 등 다른 Duty·브레이크·서보 발행 노드를 종료한다.
같은 명령 토픽에 다른 publisher가 감지되면 Start를 거부하거나 주행을 정지한다.
기본 ERPM/연결 토픽은 `auto_control.yaml`의 `measured_erpm_topic`과 `connection_status_topic`을
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
갱신하지만, 속도·IMU가 모두 유효해지기 전에는 이동 관측 기억이나 계획을 진행하지 않는다.
receive-only dynamics monitor가 이미 있으면 `dynamics_enabled:=false measured_speed_topic:=/실측속도토픽`으로 재사용한다.

ROI RGB BEV/파라미터 창과 `ROI VFH+ test` 창이 함께 열린다.
GTK HighGUI의 `WND_PROP_VISIBLE=-1`은 미지원 값이므로 창 닫힘으로 판단하지 않는다.
GTK에서는 `WND_PROP_AUTOSIZE`를 보조 조회하며, 지원되는 속성을 관측한 뒤에만
창 소멸을 판단한다. 이 호환성 수정은 빌드·테스트를 실행하지 않았다.

1. 출발 전에는 선택 경로 없음, 목표 속도/조향 0. ROI 창에서 값을 조절하고 Apply한다.
2. **Start driving**: ROI 파라미터 잠금 ACK, VESC 연결, 최신 입력을 확인한 뒤 계획과 실제 구동을 시작한다.
   프리뷰 모드는 버튼이 **Start preview**로 표시되며 액추에이터 publisher를 만들지 않는다.
3. 실행 중에는 ROI GUI 입력 및 `ros2 param set /roi_lidar ...`를 거부한다.
4. **Stop / unlock**, Space 또는 S: 구동 Duty 0, 기존 `brake_maximum_current_amps`(현재 2A) 제동,
   마지막 서보 조향 유지. 진단 경로/목표 속도·조향은 0으로 초기화하고 ROI 잠금을 해제한다.
5. Q/Esc/창 닫기도 Stop 처리한다. 센서·차선·속도·IMU·연결·잠금 유실은 정지하고 다시 Start를 요구한다.
   유효 입력에서 경로만 막힌 경우에는 제동하면서 재계획한다.

VFH 설정은 `config/preview.yaml`, 차량 모델은 기존 `auto_control`/`obstacles` YAML에서 launch 시 가져온다.
VFH 설정은 시작 시 고정이며 YAML 수정 후 재실행한다. 라이브 조절은 기존 ROI 파라미터 창을 사용한다.
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

- 일반 구간: 차선 중심 전방 방향 우선. 비용에 현재 전진 방향과 이전 선택 방향을 포함한다.
- 소량 콘: VFH+의 통과 방향에서 차량 운동/조향 변화율을 반영한 국소 궤적을 생성한다.
  관측된 장애물, 차체 외곽의 차선 밖 각각 40cm, 회전 한계를 검사한다. 미관측 셀은 통과 가능으로 취급한다.
- 유도 경로: 현재 프레임에서 양쪽 위험 경계, 장애물 없이 연결 가능한 입구, 0.35m 이상 종방향 근거를
  3개의 새 관측에서 확인한다. 확인 뒤에는 통로 중심 방향을 사용하고 40cm 제한을 해제한다.
  확인된 통로 바깥을 가로지르는 후보는 제외한다. 한쪽 경계만으로 유도 모드에 진입하지 않는다.
- 유도 구간에서 차선 중심선이 사라져도 유효한 LaneResult 메시지와 양쪽 통로 관측이 계속 있으면
  통로 방향으로 계획한다. 일반 구간의 차선 유실과 차선 노드 자체의 입력 중단은 정지한다.
- 완전히 막히면 속도 0으로 재계획한다. 탐색 범위 밖의 우회 경로 존재를 보장하지 않는다.

terrain은 요청대로 폭 **1.2m × 전방 3m**다. 허용된 40cm 중 이 격자 범위 안의 부분만 쓸 수 있다.
차선 모델은 깨끗한 **120×300 / 1cm** BEV를 받으며 HUD/점/terrain 색상은 입력하지 않는다.
범위 밖, 현재 위험 근거가 있는 셀은 계속 통과 불가다.

사용자 요청에 따라 **격자 안의 미관측(-1)은 주행 가능한 것으로 가정**한다.
히스토그램·회전 마스크·차체 검사·통로 내부 연결 검사에서 미관측을 장애물로 세지 않는다.
양쪽 유도 경계 확인에는 현재 관측된 장애물 근거가 여전히 필요하다.
원본 terrain과 관측 기억에는 가짜 바닥을 기록하지 않으며, 화면에서도 미관측은 회색이다.
따라서 표시되는 `checked length`는 미관측 통과 가정을 포함한 검사 길이이고 관측된 평지만의 길이가 아니다.
속도/IMU/차선 입력 조건은 유지한다.

## 관측 기억과 차량 모델

같은 OAK 세션에서 보정된 gyro를 읽고 EEPROM 상대 회전을 거쳐 앞차축 차량 좌표로 변환한다.
VESC 속도와 yaw rate를 적분하고 앞차축의 횡방향 이동(축간거리 × yaw rate)을 반영한다.
VESC 속도를 뒤차축 종방향 속도로 보는 초기 모델이므로 실제 속도 환산·미끄러짐 오차는 차량에서 확인한다.
이렇게 얻은 **depth 촬영 시각**의 자세로 관측을 저장·재투영한다.
차선 결과도 RGB 촬영 시각에서 해당 depth 좌표로 보정하며, 출력 경로에는 depth 촬영 시각을 기록한다.
기본 기억 한도는 0.5초 및 이동 0.35m 중 먼저 도달하는 것이다. IMU/속도 0.15초 유실 시 정지하며
시간 역전·관측 공백·정지 버튼은 기억을 초기화한다. 기억에서 가져온 셀은 수명을 갱신하지 않는다.
현재 바닥/위험 관측이 기억보다 우선한다. 단일 위험 표본도 `terrain_for_planning`에서는 차단하여
기억의 바닥으로 덮지 않는다. 움직이는 물체의 의미 추적이나 차량 추종은 이 패키지 범위에 없다.

차량 폭은 기존 `obstacles.avoidance.vehicle_half_width_m × 2`(현재 0.30m), 앞뒤 검사 길이는
`vehicle_half_length_m`(각 0.25m)을 가져온다. 실제 앞차축 위치의 비대칭 돌출 실측값이 아니라
기존 검사 모델이다. 축간거리 0.33m, 최대 조향 30°, 조향 변화율 240°/s는 기존 제어 설정을 따른다.

사용자 선택대로 **전방 관측 영역만 검사**한다. 음의 X 후방의 차체 회전 공간은 검증하지 않으며
화면·상태에 `rear unverified`로 표시한다. 격자 안의 미관측 통과 허용과 후방 미검증은 실차 모드에도 그대로 적용된다.

## 실차 연결과 선택한 여유

- `safety_margin_m=0.02`: 차체 좌우 각각 2cm. 폭 30cm 기준 명목 검사 폭 34cm이며,
  셀 교차를 빠뜨리지 않기 위한 `cell_m × 0.71` 여유가 추가된다. 앞쪽 검사에도 같은 margin을 더한다.
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

이번 실차 연결 변경은 사용자 요청에 따라 **빌드·테스트를 실행하지 않았다.**

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

기존 테스트 코드는 순수 C++ 계획/관측 기억, 프리뷰 렌더와 버튼 영역, launch 구성,
ROI 격자/스캔/프리뷰/설정 및 GUI 잠금을 검사한다. 위 테스트 명령은 수동 검증용이며 이번 변경에서는 실행하지 않았다.
ROS 전체 실행, 실제 IMU 축/속도 환산, 모델 추론 지연과 버튼-파라미터 ACK 연결은 차량 환경에서 확인한다.

## 로컬 연산 측정

재현 명령(ROS 불필요):

```bash
c++ -std=c++17 -O2 -Isrc/roi_vfh_test/include src/roi_vfh_test/test/benchmark.cpp -o /tmp/roi-vfh-benchmark
/tmp/roi-vfh-benchmark
```

1440셀의 정지 장면 4개에서 20회 워밍업 후 300회 측정한다. 기억 격자 갱신과 계획을 포함하며
카메라/차선 추론/ROS/GUI 비용은 제외한다. 전체 Jetson 실행시간은 해당 기기에서 측정해야 한다.
노드 HUD의 `plan` 시간은 계획부만 측정하며 관측 기억 갱신 비용은 제외한다.
