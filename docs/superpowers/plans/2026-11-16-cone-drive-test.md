# cone_drive_test 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 시뮬레이터의 제안 회피 알고리즘으로 실차 콘 회피를 시험하는 독립 ROS 2 패키지 `src/cone_drive_test`를 만든다.

**Architecture:** 주행 프로세스(C++ 단일 프로세스: OAK → 안정화 → CUDA BEV → 차선 → YOLO → 콘 기억·회피 계획 → 제어 → 내장 웹 서버)와
안전 전환 프로세스(Python, manual_control 복사, `/vesc/*` 유일 발행자) 2개. 순수 C++ 코어(`core/`)는 ROS·DepthAI·CUDA 없이
빌드·테스트하고, 런타임(`runtime/`)만 Jetson 전용이다.

**Tech Stack:** C++17, ROS 2 Humble(rclcpp), DepthAI 3.6, CUDA, TensorRT, OpenCV 4, cpp-httplib 0.18.3(vendored), Python 3(rclpy), 브라우저 JS(프레임워크 없음)

**Spec:** `docs/superpowers/specs/2026-11-16-cone-drive-test-design.md`

## Global Constraints

- 패키지 `src/cone_drive_test`, C++ 네임스페이스 `cone_drive_test`, Python 모듈 `cone_drive_test_py`
- 다른 워크스페이스 패키지에 빌드 의존 없음. 실행 의존은 `vesc_bridge`, `joy_initializer` launch 포함뿐(인자로 끌 수 있음)
- 기존 코드는 `git show origin/1113ML:<경로>`에서 복사하고, 파일 첫 줄 주석에 `// Copied from <원래 경로> (origin/1113ML)`를 남긴다. 동작 변경은 최소, 신호등·터널·IR·TCN 분기 제거
- 좌표계: 앞차축 기준 차량 좌표 x 앞, y 왼쪽, m. 각도 rad
- `/vesc/*`에는 안전 전환 프로세스만 발행. 주행 프로세스는 `/cone_drive/auto/{duty,brake_current,servo_position}`(std_msgs/Float32)
- 검출 모드(`mode:=detect`)는 자동 명령을 절대 발행하지 않는다
- 회피 파라미터 기본값 = `tools/avoidance_sim/avoidance_sim/config.py` 현재 값(차선 안 soft 0.06/hard 0.015, 차선 밖 0.10/0.005 포함)
- 제어 파라미터 기본값 = `origin/1113ML:src/auto_control/config/auto_control.yaml`
- 모델: `multiclass_yolox_tiny_512x320_holdout10.onnx`, `fast_scnn_stop_line_120x300_batch_1.onnx`(+json/labels/metadata)
- 웹: 기본 `0.0.0.0:8765`, **프리뷰 끄기 토글**(끄면 오버레이 좌표·단계 통계 스냅샷·사용률·영상 생성 전부 중단, 상태·모드·사유·점검 항목만 2 Hz. 시작/정지/Refit/heartbeat는 유지. 켠 클라이언트가 하나라도 있으면 생성), 텔레메트리 ≤30 Hz, BEV JPEG ≤15 fps(토글 ON + 클라이언트 연결 시만), heartbeat 200 ms, `link_loss_stop_sec` 1.0
- 프레임 큐 없음(최신 값 하나 슬롯)
- 커밋은 태스크마다, 브랜치 `claude/bold-tesla-jtg2o6`에 push

## Review Focus

1. **오래되거나 순서가 뒤바뀐 검출**(촬영 시각이 오도메트리 이력보다 오래됨, 미래 시각) → 버리고 카운트만 증가, 크래시·위치 왜곡 없음 → Task 4 테스트
2. **차선이 끊긴 상태에서 콘 기억만 남음** → 기억으로 계속 달리지 않고 watchdog으로 STOPPING → Task 6 테스트
3. **브라우저가 여러 개 접속하거나 하나만 닫힘** → 하나라도 heartbeat가 살아 있으면 유지, 모두 끊기면 STOPPING. 한 번도 연결 안 했으면 START 거부 → Task 6 테스트
4. **파라미터 일괄 적용에 잘못된 값**(NaN, 범위 밖, min≥max 쌍) → 전체 거부, 이전 값 유지, 오류 메시지에 항목 이름 → Task 7 테스트
5. **주행 중 Refit, Refit 실패** → 먼저 STOPPING·정지 확인, 실패 시 이전 보정 유지하고 READY로 → Task 6 테스트

---

## 파일 구조

```
src/cone_drive_test/
  CMakeLists.txt  package.xml  README.md
  config/cone_drive_test.yaml          # 모든 파라미터 기본값
  launch/cone_drive_test.launch.py     # 주행 노드 + 안전 노드 + vesc_bridge + joy
  models/                              # 복사한 ONNX와 메타데이터
  third_party/httplib.h                # cpp-httplib 0.18.3
  web/index.html  web/app.js  web/style.css
  include/cone_drive_test/
    copied/   (control_core.hpp, object_localization.hpp, postprocess.hpp, centerline.hpp, lane_connector.hpp,
               processing_profile.hpp, bev_geometry.hpp, startup_attitude.hpp, ground_plane_estimator.hpp,
               imu_image_stabilizer.hpp, vehicle_motion_estimator.hpp, cuda_bev_processor.hpp,
               tensorrt_lane_backend.hpp, yolox_detector.hpp, tensorrt_yolox_backend.hpp, oak_startup_measurement.hpp,
               depth_projection.hpp, ground_filter.hpp)
    core/     (types.hpp, cone_memory.hpp, frenet_planner.hpp, avoidance_planner.hpp, odometry.hpp, visibility.hpp,
               box_depth.hpp, longitudinal.hpp, steering.hpp, drive_state.hpp, params.hpp, stage_stats.hpp,
               system_monitor.hpp, telemetry.hpp)
    runtime/  (camera_pipeline.hpp, lane_worker.hpp, detect_worker.hpp, control_worker.hpp, web_server.hpp, latest.hpp)
  src/copied/*.cpp *.cu   src/core/*.cpp   src/runtime/*.cpp   src/drive_node.cpp
  cone_drive_test_py/  __init__.py handover_state.py duty_command_profile.py brake_current_profile.py
                       actuator_commander_node.py safety_node.py
  scripts/safety_node
  test/host/CMakeLists.txt             # ROS 없는 호스트 빌드 (core + 순수 copied)
  test/host/*_test.cpp  test/fixtures/*.json  test/python/test_*.py
tools/avoidance_sim/avoidance_sim/export_fixtures.py   # 골든 입력·출력 생성
tools/avoidance_sim/avoidance_sim/preview_replay.py    # 시뮬레이터 → 텔레메트리 재생 서버 (프리뷰 샘플용)
```

공통 타입 (`core/types.hpp`, Task 1에서 정의, 이후 모든 태스크가 사용):
```cpp
struct Vec2 { double x{0}, y{0}; };
struct Pose2 { double x{0}, y{0}, yaw{0}; };              // odom 좌표, 앞차축
struct ConeObservation { Vec2 center; Vec2 front; double bearing; double range; int class_id; float score;
                         double length, width; };          // 차량 좌표(관측 시점)
struct LaneObservation { std::vector<Vec2> path; std::vector<Vec2> center; std::vector<Vec2> left, right;
                         double capture_time; bool valid; };  // path=코너 바깥 이동 포함, center=미포함(없으면 빈 벡터)
struct DetectionFrame { double capture_time; std::vector<ConeObservation> cones; };
double now_sec();  // steady clock
```

---

### Task 1: 패키지 뼈대, 호스트 테스트 빌드, 순수 CPU 코드 복사

**Files:**
- Create: `src/cone_drive_test/{CMakeLists.txt,package.xml}`, `include/cone_drive_test/core/types.hpp`, `src/core/types.cpp`
- Create (복사): `copied/control_core.{hpp,cpp}`(auto_control), `copied/object_localization.hpp`, `copied/postprocess.hpp`(multiclass_yolo),
  `copied/centerline.{hpp,cpp}`, `copied/lane_connector.{hpp,cpp}`, `copied/processing_profile.{hpp,cpp}`(line_detactor),
  `copied/bev_geometry.{hpp,cpp}`(bev_processor), `copied/startup_attitude.{hpp,cpp}`, `copied/ground_plane_estimator.{hpp,cpp}`(oak_startup)
- Create: `third_party/httplib.h`, `models/*`, `test/host/CMakeLists.txt`, `test/host/control_core_test.cpp`, `test/host/localization_test.cpp`

**Interfaces:**
- Produces: CMake 타깃 `cdt_core`(core + 순수 copied, OpenCV core/imgproc만 의존). 호스트 빌드 명령
  `cmake -S src/cone_drive_test/test/host -B build/host && cmake --build build/host -j4 && ctest --test-dir build/host --output-on-failure`
- Produces: 복사 파일의 API는 원본과 동일(네임스페이스만 `cone_drive_test`)

- [ ] **Step 1:** 호스트에 `libopencv-dev` 설치(`apt-get install -y libopencv-dev`), httplib을 `https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.18.3/httplib.h`에서 받아 `third_party/`에 둔다
- [ ] **Step 2:** 위 파일들을 복사하고 네임스페이스·include 경로를 바꾼다. 모델 5개 파일(+json/labels/metadata) 복사
- [ ] **Step 3: 테스트 작성** — `control_core_test.cpp`: 원본 `test/test_control_core.py`가 검사하던 성질 중 Stanley 부호(경로가 왼쪽 y=+0.1 직선 → 조향 > 0), 직선 경로 CTE 0, `plan_path_speeds`가 반경 0.5 m 원호에서 `sqrt(a_lat/κ)`(a_lat 1.6 → 1.789)를 `maximum_speed` 1.8 안에서 넘지 않음, `erpm_to_speed_mps(0,...) == 0`.
  `localization_test.cpp`: 원본 `test/object_localization_test.cpp` 그대로 이식
- [ ] **Step 4:** 호스트 빌드·ctest → PASS
- [ ] **Step 5: ONNX 스모크** — `test/python/test_models.py`: onnxruntime CPU로 YOLO 입력 `[1,3,320,512]` 영(114) 텐서 → 출력 shape `[1,3360,9]`, 유한값. 차선 모델 `[1,3,300,120]` → `[1,3,300,120]`. `pytest -q src/cone_drive_test/test/python/test_models.py` PASS
- [ ] **Step 6:** 커밋 `feat(cone_drive_test): package skeleton, copied pure-CPU sources, host tests`

### Task 2: 시뮬레이터 골든 데이터

**Files:**
- Create: `tools/avoidance_sim/avoidance_sim/export_fixtures.py`, `src/cone_drive_test/test/fixtures/{memory.json,planner.json}`

**Interfaces:**
- Produces `memory.json`: `{"params": {...ProposedParams 메모리 항목}, "steps": [{"pose":[x,y,yaw], "detections":[[x,y],...], "expected_visible":[i,...] , "confirmed":[[x,y],...]}]}` — 노이즈 켠 시뮬레이터 1단계·2단계 실행에서 플래너에 들어간 프레임 그대로 300프레임씩
- Produces `planner.json`: `{"params": {...}, "cases": [{"name", "l0", "rear_sl":[s,l], "cones_sl":[[s,l],...], "s_end", "margin_extra", "lane_hard", "relaxed", "result": null | {"s":[...], "l":[...]}}]}` — 지그재그(간격 0.5/0.6/0.75, 침범 0.18/0.20), 유도선, 벽(차단), 빈 입력, 콘 1개 등 최소 20개 + 시뮬레이터 실행 중 무작위 표본 30개
- Produces `rollout.json`: `{"cases":[{"path":[[x,y],...], "cones_local":[[x,y],...], "speed", "steer", "ok": bool}]}` 30개

- [ ] **Step 1:** `ConeMemory.update`·`ProposedPlanner._dp`·`_rollout_ok`의 입력과 출력을 가로채 JSON으로 저장하는 스크립트 작성 (시뮬레이터 코드 변경 없이 래핑)
- [ ] **Step 2:** `python -m avoidance_sim.export_fixtures --out ../../src/cone_drive_test/test/fixtures` 실행, 파일 3개 생성 확인 (빈 결과·None 결과가 각각 1개 이상 포함)
- [ ] **Step 3:** 커밋 `test(cone_drive_test): simulator golden fixtures`

### Task 3: 콘 기억 (C++)

**Files:**
- Create: `include/cone_drive_test/core/cone_memory.hpp`, `src/core/cone_memory.cpp`, `test/host/cone_memory_test.cpp`

**Interfaces:**
- Consumes: `types.hpp`, `memory.json`
- Produces:
```cpp
struct MemoryParams { double gate, gate_radial_max, gate_lateral, merge_distance; int confirm_hits, miss_delete;
                      double delete_behind, range_noise_base, range_noise_per_m; };   // 이름은 config.py와 같게
struct TrackedCone { Vec2 world; double weight; int hits, misses; bool confirmed; std::uint32_t id; };
class ConeMemory {
 public:
  explicit ConeMemory(const MemoryParams &);
  void set_params(const MemoryParams &);
  void update(const Pose2 & pose_at_capture, const std::vector<ConeObservation> & cones,
              const std::vector<int> & expected_visible_ids);   // expected는 update 전 tracks 인덱스 → id
  void drop_behind(const Pose2 & current_pose, double rear_bumper_x);
  std::vector<Vec2> confirmed_world() const;
  const std::vector<TrackedCone> & tracks() const;
  void clear();
};
```
- 알고리즘은 `planners.py`의 `ConeMemory`(타원 게이트, 1/σ² 가중 평균, 병합, 확인 hits, miss는 보여야 할 때만)를 그대로 따른다

- [ ] **Step 1: 테스트 작성** — `memory_matches_simulator`: `memory.json`의 각 step을 순서대로 넣고 `confirmed_world()`가 기대값과 같은 개수, 각 점 1 mm 이내(정렬 후 최근접 매칭).
  `keeps_cone_after_leaving_view_and_drops_behind`: Python 테스트 `test_memory_keeps_cone_after_it_leaves_view_and_drops_it_behind_car` 이식.
  `merges_noisy_duplicates`: Python `test_memory_merges_noisy_duplicates` 이식(±6 cm → 1개)
- [ ] **Step 2:** 빌드 → 링크 실패(FAIL) 확인
- [ ] **Step 3:** 구현
- [ ] **Step 4:** ctest PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): cone memory port`

### Task 4: 회피 계획기 (Frenet DP + 롤아웃 + 단계 선택) 와 오도메트리

**Files:**
- Create: `core/frenet_planner.{hpp,cpp}`, `core/avoidance_planner.{hpp,cpp}`, `core/odometry.{hpp,cpp}`, `core/visibility.{hpp,cpp}`
- Test: `test/host/frenet_planner_test.cpp`, `test/host/avoidance_planner_test.cpp`, `test/host/odometry_test.cpp`

**Interfaces:**
- Consumes: Task 3 `ConeMemory`, fixtures
- Produces:
```cpp
struct PlannerParams { /* config.py ProposedParams의 s_max..backward_extension 전부, 같은 이름 */
                       double free_hard_clearance, free_soft_clearance; /* + VehicleParams 치수 */ };
struct FrenetPath { std::vector<double> s, l; };
std::optional<FrenetPath> frenet_dp(const PlannerParams &, double l0, Vec2 rear_sl,
    const std::vector<Vec2> & cones_sl, double s_end, double margin_extra, bool lane_hard, bool relaxed);
bool rollout_ok(const PlannerParams &, const ControlParams &, const std::vector<Vec2> & path_local,
    const std::vector<Vec2> & cones_local, double speed, double steer);
struct ControlParams { /* config.py ControlParams 중 Stanley·조향 필터·지연·rollout 관련 */ };
enum class PlanMode { Center, Lane, LaneRelaxed, Free, Fallback };
struct PlanResult { std::vector<Vec2> path_local; PlanMode mode; bool rollout_ok; double speed_cap; double compute_ms; };
class AvoidancePlanner {
 public:
  AvoidancePlanner(const PlannerParams &, const ControlParams &);
  void set_params(const PlannerParams &, const ControlParams &);
  PlanResult plan(const LaneObservation & lane, const std::vector<Vec2> & cones_world, const Pose2 & pose,
                  double speed, double steer);     // 근처 콘 없으면 Center (DP 호출 없음)
};
class Odometry {
 public:
  explicit Odometry(double wheelbase);
  void reset(const Pose2 &);
  void update(double t, double speed, double yaw_rate);         // 뒷차축 적분, 시뮬레이터 Odometry와 같은 식
  Pose2 pose() const;                                             // 앞차축
  std::optional<Pose2> pose_at(double t) const;                   // 최근 2 s 링버퍼 선형 보간, 범위 밖 nullopt
};
struct CameraGeometry { double camera_x, height, pitch_rad, hfov_rad, vfov_rad, roi_top_ratio, max_range; };
std::vector<int> expected_visible(const std::vector<Vec2> & cones_local, const CameraGeometry &, double cone_size);
```
- `AvoidancePlanner::plan`은 `ProposedPlanner.update`의 계획 부분(center 모드 판정, 단계 (lane,0)→(lane,relaxed)→(free,0), 재계획 margin `rollout_retry_margin`, 실패 시 `rollout_fail_speed`)을 따른다. 메모리 갱신은 포함하지 않는다(제어 워커가 따로 호출)
- `expected_visible`: 화각·ROI 상단 행 위/아래·max_range 안, 그리고 카메라에서 본 각도 구간이 더 가까운 콘에 80 % 이상 가리지 않음

- [ ] **Step 1: 테스트 작성** — `dp_matches_simulator`: `planner.json` 각 case에서 None 여부 동일, 결과가 있으면 같은 s 격자에서 |Δl| ≤ 0.01 m.
  `rollout_matches_simulator`: `rollout.json`에서 ok 일치율 ≥ 29/30.
  `dp_leaves_lane_only_when_blocked`, `zigzag_stays_in_lane`: Python 테스트 이식.
  `planner_center_mode_without_cones`: 콘 없음 → mode Center, path == lane.path.
  `dp_runtime`: 지그재그 case 100회 평균 < 3 ms(호스트 -O2).
  `odometry_pose_at_interpolates_and_rejects_out_of_range` (**Review Focus 1**): 0~2 s 기록, `pose_at(-0.1)`·`pose_at(미래+0.05)`·2 s보다 오래된 시각 → nullopt, 중간 시각 보간 오차 < 1 mm.
  `hidden_cone_not_expected`: (1.0,0), (1.4,0) → 1번만 expected 아님
- [ ] **Step 2:** FAIL 확인
- [ ] **Step 3:** 구현 (`-O2`, 할당 재사용: DP 비용 배열을 멤버로 보관)
- [ ] **Step 4:** PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): frenet planner, rollout, odometry port`

### Task 5: 박스 깊이 → 콘 위치, 종방향·조향 처리

**Files:**
- Create (복사): `copied/depth_projection.hpp`, `copied/ground_filter.hpp`(point_cloud, 필요한 함수만)
- Create: `core/box_depth.{hpp,cpp}`, `core/longitudinal.{hpp,cpp}`, `core/steering.{hpp,cpp}`
- Test: `test/host/box_depth_test.cpp`, `test/host/longitudinal_test.cpp`

**Interfaces:**
- Produces:
```cpp
struct DepthIntrinsics { double fx, fy, cx, cy; int width, height; };       // depth 영상(RGB 정렬 후) 기준
struct GroundModel { /* 시작 측정 결과: 높이, roll, pitch, 카메라→차량 회전·위치 */ };
struct BoxDepthOptions { double box_core_fraction{0.6}; int pixel_stride{1}; double min_height, max_height; LocalizationOptions loc; };
std::optional<ConeObservation> localize_box(const std::uint16_t * depth_mm, int stride_px, const DepthIntrinsics &,
    const GroundModel &, const cv::Rect2f & box_in_depth, int class_id, float score, const Footprint &, const BoxDepthOptions &);
struct LongitudinalParams { /* auto_control.yaml의 longitudinal_*, duty_*, brake_*, obstacle_*, minimum/maximum_speed 등 */ };
struct LongitudinalCommand { double duty; double brake_current; double target_speed; std::string phase; };
class LongitudinalController {     // auto_control_node의 단계 제어(duty 감속→전류 제동→회복, 시작 duty, 장애물 램프) 이식
 public:
  explicit LongitudinalController(const LongitudinalParams &);
  void set_params(const LongitudinalParams &);
  void reset();
  LongitudinalCommand update(double dt, double current_speed, const PathSpeedPlan & plan,
                             bool obstacle_active, double obstacle_speed_cap, bool stop_requested);
};
struct SteeringParams { double max_steer_rad, steering_current_weight, rate_limit_rad_s, servo_left, servo_center, servo_right; bool inverted; };
class SteeringFilter { public: explicit SteeringFilter(const SteeringParams &); double update(double raw, double dt); double servo(double angle) const; void hold(); };
```
- `localize_box`는 박스 가운데 `box_core_fraction` 픽셀만 역투영(점이 6개 미만이면 박스 전체), 지면 위 높이 필터 후 `localize()` 호출. 전체 깊이 영상을 점구름으로 만들지 않는다

- [ ] **Step 1: 테스트 작성** — `box_depth_flat_cone`: 합성 깊이 영상(지면 평면 + 1.2 m 앞 콘 앞면 13 cm × 18 cm 직사각형)과 박스 → 중심 x 1.2+0.065±0.02, y ±0.02.
  `box_depth_rejects_ground_only`: 박스 안이 지면뿐 → nullopt.
  `longitudinal_start_duty_once`: 정지(v=0), 목표 1.0 → 첫 duty == `longitudinal_start_duty`(0.053), 이후 `duty_rise_rate_per_sec`로 증가.
  `longitudinal_brakes_when_overspeed`: v=1.6, 목표 1.0, coast 감속 관측 0 → `obstacle_coast_probe_sec`(0.20 s) 뒤 brake_current > 0, duty == 0.
  `longitudinal_stop_request_brakes`: stop_requested → duty 0, brake > 0 (v>0.05)
- [ ] **Step 2:** FAIL 확인
- [ ] **Step 3:** 구현 — `LongitudinalController`는 `origin/1113ML:src/auto_control/src/auto_control_node.cpp`의 종방향 단계 제어 함수들을 신호등·정지선 분기를 뺀 채 옮긴다
- [ ] **Step 4:** PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): box depth localization, longitudinal and steering control`

### Task 6: 주행 상태기계

**Files:**
- Create: `core/drive_state.{hpp,cpp}`, Test: `test/host/drive_state_test.cpp`

**Interfaces:**
- Produces:
```cpp
enum class DriveMode { Detect, Drive };
enum class DriveState { Boot, Inspecting, Ready, Driving, Stopping, Manual, Refitting };
struct Readiness { bool attitude_ok, lane_model_ready, yolo_model_ready, vesc_connected, erpm_fresh, lane_fresh, client_alive; };
struct StateEvent { std::string reason; };
class DriveStateMachine {
 public:
  explicit DriveStateMachine(DriveMode initial, double link_loss_stop_sec);
  // 요청 (웹/조이스틱). 반환: 수락 여부 + 거부 사유
  std::pair<bool, std::string> request_start(const Readiness &, double now);
  void request_stop(const std::string & reason, double now);
  std::pair<bool, std::string> request_refit(double now);
  std::pair<bool, std::string> request_mode(DriveMode, double now);
  void on_heartbeat(const std::string & client_id, double now);
  void on_inspection_done(bool ok, double now);  void on_refit_done(bool ok, double now);
  void on_handover_mode(const std::string & safety_mode);    // 안전 프로세스가 알린 AUTO/STOPPING/MANUAL
  void tick(const Readiness &, double speed, double now);     // 고장 감시, 정지 확인(|v|<0.05 0.5 s), heartbeat 만료
  DriveState state() const; DriveMode mode() const; bool auto_output_allowed() const;
  bool wants_manual_handover() const; bool wants_auto_rearm() const; bool refit_pending() const;
  std::string last_reason() const;
};
```

- [ ] **Step 1: 테스트 작성**
  `detect_mode_never_allows_auto`: Detect에서 request_start → 거부 "detect mode", `auto_output_allowed()` 항상 false.
  `start_requires_all_readiness`: Readiness 항목 하나씩 false → 거부, 사유에 항목 이름.
  `start_refused_without_any_client` (**Review Focus 3**): heartbeat 한 번도 없음 → 거부.
  `multi_client_heartbeat` (**Review Focus 3**): 클라이언트 a,b 접속, a만 1.5 s 끊김 → Driving 유지, b도 끊김 1.0 s 초과 → Stopping.
  `lane_loss_while_driving_stops` (**Review Focus 2**): Driving 중 `lane_fresh=false` tick → Stopping, 사유 "lane stale". 기억 콘 유무와 무관.
  `stopping_to_manual_after_standstill`: Stopping, 속도 0.03이 0.5 s 유지 → `wants_manual_handover()` true, on_handover_mode("MANUAL") → Manual.
  `no_auto_restart_after_recovery`: Stopping 후 Readiness 모두 복구 → 상태 유지(자동 재출발 없음).
  `refit_while_driving_stops_first` (**Review Focus 5**): Driving에서 request_refit → Stopping, 정지 확인 → Refitting. `on_refit_done(false)` → Ready, last_reason에 "refit failed".
  `mode_change_only_when_stopped`: Driving에서 request_mode → 거부, Manual/Ready에서 수락
- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): drive state machine`

### Task 7: 파라미터 레지스트리, 단계 통계, 시스템 사용률, 텔레메트리 JSON

**Files:**
- Create: `core/params.{hpp,cpp}`, `core/stage_stats.{hpp,cpp}`, `core/system_monitor.{hpp,cpp}`, `core/telemetry.{hpp,cpp}`
- Test: `test/host/params_test.cpp`, `test/host/telemetry_test.cpp`

**Interfaces:**
- Produces:
```cpp
struct ParamSpec { std::string name, group /* "obstacle"|"driving" */, label, unit; double min, max, value; bool live; };
class ParamRegistry {
 public:
  void add(ParamSpec);  void add_order_constraint(std::string lower, std::string upper);   // lower < upper
  std::vector<ParamSpec> list() const;  double get(const std::string &) const;
  std::pair<bool, std::string> apply(const std::map<std::string, double> & changes);   // 원자적. 실패 시 아무것도 안 바뀜
  std::string to_yaml() const;  std::uint64_t version() const;
};
class StageStats {   // 스레드 안전, 단계별 최근 1 s
 public: void record(const char * stage, double start_sec, double end_sec);
         struct Row { std::string stage; double avg_ms, fps; }; std::vector<Row> snapshot(double now) const; };
struct SystemUsage { double cpu_percent, process_cpu_percent, mem_percent; std::optional<double> gpu_percent; };
class SystemMonitor { public: explicit SystemMonitor(std::string proc_root = "/proc", std::string sys_root = "/sys");
                      SystemUsage sample(); };   // GPU: sys_root/devices/platform/gpu.0/load, devices/gpu.0/load (0~1000)
std::string telemetry_json(const TelemetryFrame &);   // TelemetryFrame: 상태·모드·차량 수치·차선/경로/검출/기억/롤아웃·stats·usage
```
- JSON 필드(웹과 공유): `state, mode, reason, readiness{...}, inspection{attitude{status,height_m,roll_deg,pitch_down_deg,message}, models[{name,status /*WAITING|LOADING|READY|FAILED*/,stage,elapsed_s,message}]}, vehicle{speed,target,steer_deg,cte,duty,brake}, lane{left,right,center,path}` (각 `[[x,y],...]`, 최대 40점),
  `plan{path, mode, rollout_ok}`, `detections[{cls,score,x,y,length,width,yaw}]`, `memory[{x,y,confirmed,id}]`, `stages[{name,ms,fps}]`, `usage{cpu,proc_cpu,mem,gpu}`, `params_version`, `t`

- [ ] **Step 1: 테스트 작성** — `apply_rejects_invalid_atomically` (**Review Focus 4**): {a: NaN} / {a: max+1} / {min_speed: 2.0, max_speed: 1.5} 각각 → false, 메시지에 이름 포함, 모든 값 이전과 동일, version 불변. 정상 변경 → version+1.
  `yaml_roundtrip`: to_yaml 결과를 파싱(간단한 `key: value` 줄)해 값 일치.
  `stage_stats_window`: 1 s 안 10 ms 기록 30회 → avg 10, fps 100(=1000/avg).
  `system_monitor_reads_fake_proc`: 임시 디렉터리에 가짜 `/proc/stat`·`/proc/meminfo`·`gpu.0/load`(500) 작성 → mem·gpu 50 %, 두 번째 sample의 cpu % 계산 확인. gpu 파일 없음 → nullopt.
  `telemetry_has_fields_and_decimates`: 차선 200점 → JSON 안 40점 이하, 필드 모두 존재
- [ ] **Step 2:** FAIL → **Step 3:** 구현 (JSON은 손으로 작성한 작은 writer, 외부 라이브러리 없음) → **Step 4:** PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): params, stage stats, system monitor, telemetry`

### Task 8: 웹 서버와 브라우저 프리뷰, 프리뷰 샘플 이미지

**Files:**
- Create: `runtime/web_server.{hpp,cpp}`, `web/{index.html,app.js,style.css}`, `test/host/web_server_test.cpp`
- Create: `tools/avoidance_sim/avoidance_sim/preview_replay.py`, `sim_output/preview/*.png`

**Interfaces:**
- Consumes: Task 7 `telemetry_json`, `ParamRegistry`, Task 6 요청 함수(콜백으로 주입)
- Produces:
```cpp
struct WebCallbacks { std::function<std::pair<bool,std::string>()> start, refit;  std::function<void()> stop;
                      std::function<std::pair<bool,std::string>(std::string)> mode;
                      std::function<std::pair<bool,std::string>(std::map<std::string,double>)> params;
                      std::function<void(std::string)> heartbeat; std::function<void(bool)> bev_image; std::function<void(bool)> camera_image; };
class WebServer {
 public:
  WebServer(std::string web_root, std::string bind, int port, const ParamRegistry &, WebCallbacks);
  void start(); void stop();
  void publish_telemetry(std::string json);              // 최신 1개 교체, SSE 구독자에게 ≤30 Hz
  void publish_jpeg(const char * stream, std::vector<std::uint8_t> jpeg);   // "bev" | "camera"
  int client_count() const;  bool stream_wanted(const char * stream) const;
};
```
- HTTP: `GET /` 정적, `GET /api/events`(SSE), `GET /api/stream/bev`·`/api/stream/camera`(MJPEG multipart), `GET /api/params`,
  `POST /api/{start,stop,refit,mode,params,params/save,heartbeat,view}` JSON 본문 (`view`: `{client, preview?, bev?, camera?}`, 클라이언트별로 기억해 heartbeat 만료 시 해제), 응답 `{"ok":bool,"reason":str}`
- 화면은 스펙 6.9 배치. 상단 점검 영역에 카메라 자세(측정 중/PASS/FAIL, 높이·roll·pitch)와 모델별 로딩 단계·경과 시간 표시(INSPECTING 동안 크게, 이후 한 줄 요약). BEV 캔버스: 차량 좌표 x 0~3 m 위쪽, y ±0.6 m, 1 cm = 2 px(240×600 표시). 좌 차선 파랑, 우 빨강, 중앙선 노랑, 회피 경로 청록(롤아웃 실패 빨강),
  관측 콘 주황 실선 사각형 + 점수, 기억 콘 분홍 점선, 차량 외곽, 0.5 m 격자. BEV 원본 토글 시 `<img>` MJPEG를 캔버스 아래에 깔고 오버레이는 항상 위
- 검출 모드: 카메라 보기(원본 + 박스 + ROI 선, 서버가 그린 JPEG) 패널 추가, 검출 목록(클래스·점수·X,Y)
- 파라미터 창: 버튼으로 여닫는 패널, 탭 [장애물] [주행], `/api/params` 목록으로 입력 칸 생성, live=false 항목에 "재시작 필요" 표시, Apply → 결과 표시, [저장]
- heartbeat: 페이지가 200 ms마다 `client_id`(localStorage, try/catch) 포함 POST

- [ ] **Step 1: 테스트 작성** — `web_server_test.cpp`(호스트, httplib 클라이언트): `/api/params` JSON에 등록 항목 존재, `POST /api/params` 잘못된 값 → `ok:false`, `/api/events` 첫 이벤트에 publish한 JSON 포함, `POST /api/heartbeat` → 콜백 호출, `stream_wanted("bev")`는 `POST /api/view {"bev":true}` 후 MJPEG 클라이언트가 붙었을 때만 true
- [ ] **Step 2:** FAIL → **Step 3:** 구현 → **Step 4:** PASS
- [ ] **Step 5:** `preview_replay.py`: 시뮬레이터 `run(..., record=True)` 프레임을 같은 JSON 형식으로 바꿔 SSE로 재생하고, 같은 `web/` 파일과 API(stub 응답)를 제공. BEV 원본은 시뮬레이터 트랙을 BEV로 그린 JPEG
- [ ] **Step 6:** Playwright(deviceScaleFactor 2)로 캡처: `drive_lane.png`(주행 모드, 지그재그 회피 중, 오버레이만), `drive_bev.png`(BEV 원본 ON), `params.png`(파라미터 창), `detect.png`(검출 모드, 카메라 보기). 사용자에게 전송
- [ ] **Step 7:** 커밋 `feat(cone_drive_test): embedded web preview and replay samples`

### Task 9: 안전 전환 프로세스 (Python)

**Files:**
- Create (복사): `cone_drive_test_py/{handover_state.py,duty_command_profile.py,brake_current_profile.py,actuator_commander_node.py}`(manual_control)
- Create: `cone_drive_test_py/safety_node.py`(auto_handover_node 기반), `scripts/safety_node`, `test/python/test_handover_state.py`

**Interfaces:**
- Produces 토픽: 구독 `/cone_drive/auto/{duty,brake_current,servo_position}`, `/cone_drive/handover/request` (std_msgs/String: `"manual"` | `"auto"`), `/joy`, `/vesc/measured_erpm`, `/vesc/connected`;
  발행 `/vesc/{duty,brake_current,servo_position}`, `/cone_drive/handover/mode`(String, transient local: AUTO|STOPPING|MANUAL)
- `HandoverState` 확장(원본 동작 유지 + 추가): `__init__(..., initial_mode="AUTO")`, `request_manual()` (AUTO → STOPPING), `request_auto(now, neutral) -> bool` (MANUAL에서만, 정지 확인·neutral일 때 True → AUTO),
  `on_auto_command(now)` / `auto_command_stale(now, timeout)` (AUTO에서 자동 명령이 `auto_command_timeout_sec` 0.3 s 넘게 없으면 제동 출력)
- 파라미터 `initial_mode` 기본 "MANUAL": 두 모드 모두 MANUAL로 시작(조이스틱 주행 가능). drive 모드에서 START가 승인되면 주행 프로세스가 `"auto"`를 요청하고, 안전 프로세스가 수락해 mode=AUTO를 알린 뒤에야 주행 상태기계가 Driving이 된다

- [ ] **Step 1: 테스트 작성** — `test_x_blocks_auto`: AUTO에서 X 상승 → STOPPING, 이후 자동 명령 무시(전달 함수가 False).
  `test_request_auto_only_when_stopped_and_neutral`: MANUAL, ERPM 500 → request_auto False; ERPM 0 0.5 s + neutral → True.
  `test_auto_command_timeout_brakes`: AUTO, 마지막 명령 0.31 s 전 → stale True.
  `test_initial_manual`: initial_mode="MANUAL" → mode MANUAL. rclpy 없이 상태 클래스만 테스트
- [ ] **Step 2:** `pytest -q src/cone_drive_test/test/python/test_handover_state.py` FAIL → **Step 3:** 구현 → **Step 4:** PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): safety handover process`

### Task 10: GPU·DepthAI 코드 복사 (Jetson 전용)

**Files:**
- Create (복사): `copied/imu_image_stabilizer.{hpp,cpp}`, `copied/vehicle_motion_estimator.{hpp,cpp}`(camera_driver), `copied/cuda_bev_processor.{hpp,cu}`(bev_processor),
  `copied/tensorrt_lane_backend.{hpp,cpp}`, `copied/cuda_lane_pipeline.{hpp,cu}`(line_detactor), `copied/yolox_detector.{hpp,cpp}`, `copied/tensorrt_yolox_backend.{hpp,cpp}`,
  `copied/cuda_yolox_preprocessor.{hpp,cu}`, `copied/yolox_geometry.hpp`(multiclass_yolo), `copied/oak_startup_measurement.{hpp,cpp}`(oak_startup), stereo 구성 함수(point_cloud `addStereoDepth`)
- Modify: `test/host/CMakeLists.txt` — `imu_image_stabilizer`, `vehicle_motion_estimator`는 OpenCV만 쓰므로 호스트 빌드에 포함하고 원본 테스트(`camera_driver/test/imu_image_stabilizer_test.cpp`) 이식

**Interfaces:**
- Produces: 원본과 같은 API(네임스페이스만 변경). `YoloxDetector`는 CPU(`cv::dnn`) 경로도 유지 → 호스트에서 ONNX 실행 가능

- [ ] **Step 1:** 복사·네임스페이스 변경, 터널·IR·노출 복구 관련 분기 제거
- [ ] **Step 2:** 호스트 빌드에 `yolox_detector`(TensorRT OFF 경로, `tensorrt_disabled.cpp` 복사) 포함, 원본 `postprocess_test.cpp`·`model_smoke.cpp` 이식 → ctest PASS
- [ ] **Step 3:** 안정화 테스트 이식 → PASS
- [ ] **Step 4:** CUDA/DepthAI 파일은 호스트에서 `g++ -fsyntax-only`가 불가하므로 include·네임스페이스 일관성만 `grep`으로 확인(원래 네임스페이스 잔존 0)
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): copy GPU, TensorRT and DepthAI sources`

### Task 11: 런타임 통합 (주행 노드, 워커, launch, 설정, README)

**Files:**
- Create: `runtime/latest.hpp`, `runtime/camera_pipeline.{hpp,cpp}`, `runtime/lane_worker.{hpp,cpp}`, `runtime/detect_worker.{hpp,cpp}`, `runtime/control_worker.{hpp,cpp}`, `src/drive_node.cpp`
- Create: `config/cone_drive_test.yaml`, `launch/cone_drive_test.launch.py`, `README.md`
- Modify: `CMakeLists.txt`, `package.xml`

**Interfaces:**
- Consumes: Task 1~10 전부
- Produces:
  - `template<class T> class Latest { void put(std::shared_ptr<const T>); std::shared_ptr<const T> get() const; std::shared_ptr<const T> wait_newer(std::uint64_t seq, double timeout); }`
  - 모델 로딩은 별도 스레드에서 두 엔진을 순서대로 만들고 progress 콜백으로 `inspection.models`를 갱신, 끝나면 시험 추론 1회 후 READY
  - `CameraPipeline`: 시작 측정(`measureOakStartupExtrinsics`) → 본 파이프라인(CAM_A NV12 1280×800 `sensor_fps` 40, undistort, Stereo 400p, IMU 400 Hz, 모든 큐 크기 1 non-blocking) → `Latest<CameraFrame>`(NV12 + 시각 + 짝지은 depth + 안정화 보정). `refit()`는 파이프라인 닫기 → 측정 → LUT·기준 갱신 → 재시작
  - `LaneWorker`: 새 CameraFrame마다 CUDA BEV(GPU) → `infer_device_bgr` → lane_connector → centerline → `Latest<LaneObservation>`. 콘이 근처에 있다는 플래그(제어 워커가 설정)일 때만 바깥 이동 끈 중앙선 추가 계산. BEV JPEG는 `stream_wanted("bev")`일 때만 다운로드·인코딩(≤15 fps)
  - `DetectWorker`: 최신 CameraFrame만, 상한 FPS, `detect_nv12`(ROI) → `localize_box` → `Latest<DetectionFrame>`. 카메라 JPEG는 `stream_wanted("camera")`일 때만
  - `ControlWorker`: LaneObservation마다 오도메트리(ERPM 속도 + IMU 자이로) → 새 DetectionFrame이 있으면 `pose_at(capture_time)`로 기억 갱신(없으면 버림, 카운트) → 계획 → `plan_path_speeds` → `stanley_control` → `SteeringFilter` → `LongitudinalController` → 상태기계 `auto_output_allowed()`일 때만 비공개 토픽 발행. 텔레메트리 생성은 클라이언트가 있을 때만, ≤30 Hz
  - 모든 단계 `StageStats::record`: `receive, stabilize_bev, lane_infer, lane_post, yolo_pre, yolo_infer, yolo_post, box_depth, memory_plan, control, web`
- launch 인자: `mode`(drive|detect, 기본 detect), `params_file`, `vesc_enabled`(true), `joy_enabled`(true), `web_port`(8765), `vehicle_namespace`(기존 manual_drive와 같은 기본값) — vesc_bridge·joy_initializer는 원래 launch를 `IncludeLaunchDescription`
- YAML: Task 4·5·7 파라미터 전부, 기본값은 Global Constraints 출처 그대로

- [ ] **Step 1:** 워커·노드·launch·YAML 작성. `CMakeLists.txt`에 CUDA/TensorRT 옵션(`CONE_DRIVE_TEST_TENSORRT`, 기본 ON), web·models·config·launch install, Python 모듈 `ament_python_install_package`, `scripts/safety_node` install
- [ ] **Step 2:** 호스트에서 할 수 있는 검사: `python3 -c "import ast; ast.parse(open('launch/cone_drive_test.launch.py').read())"`, `xmllint --noout package.xml`, YAML 파싱(`python3 -c "import yaml; yaml.safe_load(...)"`), YAML 키가 `ParamRegistry` 등록 이름과 일치하는지 스크립트로 확인
- [ ] **Step 3:** README: 빌드 명령, 실행(검출/주행), 브라우저 주소, 화면 설명, 안전 동작(X 버튼, heartbeat, START 조건), 파라미터, Jetson 첫 실행 점검 순서(①모델 엔진 생성 확인 ②정지 상태 자세 측정 PASS ③검출 모드로 콘 위치 확인 ④바퀴 들고 drive START → 조향·duty 확인 ⑤저속(`maximum_speed` 0.6) 주행), 이 환경에서 검증한 것과 못 한 것
- [ ] **Step 4:** 전체 호스트 ctest + pytest 다시 실행 → PASS
- [ ] **Step 5:** 커밋 `feat(cone_drive_test): runtime integration, launch, config, README`

---

## 실행 순서와 의존
Task 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11. Task 9·10은 1 이후 어느 때나 가능.
