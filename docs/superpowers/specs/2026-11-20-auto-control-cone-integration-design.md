# auto_control ← cone_drive_test 기능 통합 설계

작성일: 2026-11-20 · 브랜치 `1113ML`

## 목표

`vehicle_bringup auto_drive.launch.py`(auto_control 시스템)를 기반으로, `cone_drive_test`에서 실차 검증한
기능(웹 GUI, 멀티클래스 YOLO, 콘 기억·회피, 신호등 정지, 앞차 추종, duty_only 종방향 제어, IMU 저주파 보정,
녹화·분석 도구)을 auto_control 안으로 옮긴다. 작업 후 `cone_drive_test` 패키지를 지워도 빌드·실행에 영향이 없어야 한다.

## 확정된 결정 (사용자 답변)

| 항목 | 결정 |
|---|---|
| 기반 | auto_control 시스템. 카메라·BEV·차선은 기존 파이프라인(camera_driver → bev_processor → line_detactor) 유지 |
| 제어 판단부 | cone_drive의 `ControlLoop`(콘 기억·Frenet 계획·롤아웃·신호등·앞차 추종·duty_only·상태기계)로 교체. `auto_control_node.cpp` 삭제 |
| 프로세스 구조 | 연산 최적 쪽: 새 주행 컴포넌트를 카메라/BEV/차선 컨테이너에 함께 로드 (영상·깊이·차선 결과를 복사 없이 공유) |
| GUI | `remote_preview`(Python) 폐기 → cone 스타일 C++ 웹(포트 8765). 신호등·주행 상태·출발 점검·터널 상태 표시 추가 |
| 실시간 파라미터 | 주행·신호등·YOLO 주요 항목만 (cone 파라미터 창의 [장애물] [주행] 탭 그대로) |
| 신호등 | `traffic_detection_test` 노드·모델 폐기 → 멀티클래스 YOLO의 `traffic_light` + HSV 색 판정, cone 정지선 로직 |
| 종방향 | cone `duty_only`로 교체. auto_control의 전류 단계 제동·signed PID·곡률 감속 제거 |
| 회피 | auto_control 깊이 기반 장애물 검출(`ObstacleDetectorNode`, `AvoidancePlan`) 제거 → cone 콘 기억 + Frenet + 롤아웃 |
| YOLO ROI | 넓혀서 한 번 추론으로 콘·차량·신호등 함께 검출 (ROI 높이 %는 GUI 실시간) |
| 추가 이식 | 앞차 추종(truck·sedan), IMU 저주파 보정, 녹화·분석 도구 |
| 출발 | 조이스틱을 먼저 조작하지 않아도 [주행 시작] 가능. 조이스틱이 연결 안 돼 있어도 허용 |
| 삭제 | VFH 시뮬레이터(`auto_control/tools/vfh_sim`) |

## 설계 중 정한 사항 (검토 필요)

1. **안전 전환은 cone의 `safety_node`로 바꾼다.** 기존 `auto_handover`는 AUTO로 시작해서 X를 누르면 MANUAL로 한 번만
   넘어가는 단방향 구조라, cone의 [정지] → [주행 시작] 반복(정지 후 MANUAL, 다시 AUTO)을 받을 수 없다.
   cone `safety_node`를 복사하되 "조이스틱 한 번 켜야 주행" 문제의 원인인 **AUTO 승인 때 조이스틱 중립 입력 필수** 조건을 바꾼다:
   - 조이스틱 입력이 최근 `input_timeout`(기본 0.5 s) 안에 없으면(미연결·무입력) → 중립으로 보고 AUTO 승인
   - 조이스틱이 연결돼 입력이 오고 있으면 → 지금처럼 페달·조향이 중립일 때만 승인 (RT를 밟은 채 AUTO로 넘어가지 않게)
   - X 버튼 차단, 자동 명령 0.3 s 끊김 제동, 정지 확인 후 MANUAL은 그대로
2. **차선 경계 전달**: cone 회피 계획은 좌우 차선 경계가 필요한데 `LaneResult`에는 중앙선만 있다.
   `line_detactor`가 이미 계산하는 관측 경계(observed_paths)와 코너 바깥 이동 없는 기준 중앙선을 `LaneResult`에 추가한다
   (`left_points`, `right_points`, `reference_centerline_points`). 추가 연산은 기준 중앙선 1회 생성뿐.
   cone의 차선 좌우 재판정(`lane_side_tracker`)도 line_detactor 옵션으로 옮긴다(기본 켬, cone과 같음).
3. **깊이 정렬**: cone 박스 위치 추정·보정값(`object.range_gain` 등)은 CAM_A에 정렬된 깊이 기준이다.
   현재 camera_driver의 깊이를 쓰는 곳은 지금 지우는 장애물 검출뿐이라, camera_driver 깊이를 `setDepthAlign(CAM_A)`로 바꾼다
   (장치에서 정렬하므로 Jetson 연산 추가 없음). 크기는 cone과 같게.
4. **IMU·오도메트리**: camera_driver의 IMU 브리지(`/camera/imu`)를 같은 프로세스에서 받아 자이로 요레이트로 쓴다.
   ERPM은 기존 VESC 토픽. IMU 저주파 보정(차 가속도 제거)은 camera_driver 영상 안정화에 옵션으로 넣고,
   필요한 속도는 ERPM 토픽을 camera_driver가 직접 구독한다.
5. **터널(IR) 구간**: `lane_source == "IR"`이면 YOLO 결과를 쓰지 않는다 → 콘 회피·앞차 추종·신호등 끔, 차선 추종만.
   IR 차선 결과에는 경계가 없어도 된다(중앙선만으로 Stanley).
6. **출발 전 점검**: auto_control의 Python 점검 창(`startup_inspection`)·START 서비스 대신 cone 점검 영역을 쓴다.
   항목: 카메라 자세(bev_processor 시작 측정 결과), 차선·YOLO 모델 READY, VESC·ERPM, 차선, 브라우저 연결 (+ 터널 켠 경우 IR 모델).
7. **버리는 auto_control 기능**: 전류 단계 제동, 신호등 "못 서면 통과"·임시 정지, 곡률 감속, line_detactor 속도 제한 패널 요청
   (`runtime_speed_limit`), 장애물 감속, 성능 측정 자동 종료, 데이터셋 수집, Python 원격 프리뷰, 시작 점검 창.
   주행 로그는 cone의 콘 관측 CSV·수동 녹화로 대체.

## 구조

```
[bev_processor_container (한 프로세스)]
 camera_driver  ── NV12(CAM_A 전체) ──┬─ bev_processor → line_detactor ─ LaneResult(intra) ─┐
     │  깊이(CAM_A 정렬) · IMU         │                   (터널: TunnelLaneSelector 경유)    │
     │                                 └─ DirectCameraConsumer ─┐                           │
     ▼                                                          ▼                           ▼
 auto_control::AutoDriveNode (새 컴포넌트)
   ├ 검출 스레드: YOLOX(NV12 CUDA 전처리, TensorRT) → 박스 깊이 → 콘·차량 위치, 신호등 HSV 색
   ├ 제어: LaneResult마다 ControlLoop.on_lane (차선 변환 → 콘 기억 → 회피 → Stanley·duty_only → 상태기계)
   ├ 감시 타이머: ControlLoop.tick (차선 끊김 등)
   ├ 웹 서버: SSE 텔레메트리, BEV(direct BEV handoff)·카메라 MJPEG(켰을 때만), 명령·파라미터
   └ 출력: <ns>/auto/{duty,brake_current,servo_position}, 안전 전환 요청
[safety_node (Python, 별도 프로세스)]  ← vesc/*의 유일한 발행자 (수동 주행 포함)
[vesc_bridge, joy_initializer]
```

## 파일 배치 (auto_control)

- `include/auto_control/drive/{core,copied,runtime}/`, `src/drive/{core,copied,runtime}/`: cone 코드 복사
  (첫 줄에 출처 주석). 네임스페이스 `cone_drive_test` → `auto_control::drive`.
- `src/drive/auto_drive_node.cpp`: 컴포넌트 (cone `drive_node.cpp`에서 카메라·차선 모델 부분을 빼고 LaneResult·handoff 입력으로 바꿈)
- `web/`: cone `web/` + 신호등·터널·점검 표시
- `auto_control/` (Python): `safety_node`, `handover_state` 등 cone 것 복사
- `models/`: 멀티클래스 YOLOX ONNX·메타데이터
- `tools/`: `analyze_cone_obs.py`, `analyze_manual.py`, `analyze_layout.py`, `fit_planner.cpp`
- `test/host/`: cone 호스트 테스트 (ROS 없이 `cmake`로 빌드)
- 삭제: `src/auto_control_node.cpp`, 장애물·회피 소스와 `msg/AvoidancePlan.msg`, `auto_control_tunnel/remote_*`·`startup_inspection`
  (터널 선택기 관련은 유지), Python 구현 참고용 파일, `tools/vfh_sim`

## 다른 패키지 변경

- `line_detactor`: `LaneResult`에 경계·기준 중앙선 추가, 차선 좌우 재판정 옵션
- `camera_driver`: 깊이 CAM_A 정렬, IMU 저주파 보정 옵션
- `vehicle_bringup/auto_drive.launch.py`: 신호등·장애물 노드·`auto_control_node`·원격 프리뷰·점검 창·`auto_handover` 제거,
  `AutoDriveNode` 컴포넌트·`safety_node` 추가. 주요 인자(`vehicle_namespace`, `tunnel_enabled`, `vesc_port` …) 유지
- `traffic_detection_test`, `multiclass_yolo`, `cone_drive_test`: 수정하지 않음 (의존만 끊음)

## 검증

- 이 환경(ROS·CUDA·TensorRT 없음): cone 호스트 테스트를 auto_control 경로에서 그대로 통과, 새 차선 변환·안전 전환
  (조이스틱 없음 → AUTO 승인) 테스트 추가, 웹 JS 테스트, Python 테스트, 런치 파일 문법 검사
- Jetson에서 확인 필요: colcon 빌드, 엔진 생성, 깊이 정렬 위치, FPS, 실차 주행
