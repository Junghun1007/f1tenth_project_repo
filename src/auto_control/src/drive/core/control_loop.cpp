// Copied from src/cone_drive_test/src/core/control_loop.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/control_loop.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/visibility.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_control::drive {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
}

// 계획에 넣을 장애물 칸: 콘은 그대로, 차량(트럭·세단)은 크기만큼 콘 크기 이하 간격의 정사각형 칸으로 채운다
// (차량 방향은 박스로 부정확하므로 차량 좌표 축에 정렬 = 차선 위에 서 있다고 가정). 계획기·롤아웃은 칸을 콘처럼 다룬다
std::vector<ConeObservation> obstacle_cells(const std::vector<ConeObservation> & obs, double cone, bool vehicles) {
  std::vector<ConeObservation> out;
  for (const auto & c : obs) {
    if (c.class_id == 0) { out.push_back(c); continue; }
    if (!vehicles || (c.class_id != 1 && c.class_id != 2) || !(c.length > cone) || !(c.width > cone)) continue;
    const int nx = int(std::ceil((c.length - cone) / cone - 1e-9)) + 1, ny = int(std::ceil((c.width - cone) / cone - 1e-9)) + 1;
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j) {
        ConeObservation q = c;
        q.class_id = 0;
        q.center.x = c.center.x - (c.length - cone) / 2 + (nx > 1 ? i * (c.length - cone) / (nx - 1) : (c.length - cone) / 2);
        q.center.y = c.center.y - (c.width - cone) / 2 + (ny > 1 ? j * (c.width - cone) / (ny - 1) : (c.width - cone) / 2);
        out.push_back(q);
      }
  }
  return out;
}

ControlLoop::ControlLoop(DriveMode mode, const RuntimeConfig & rc)
  : rc_(rc), sm_(mode, 1.0), memory_(MemoryParams{}), planner_(PlannerParams{}, ControlParams{}), odom_(rc.wheelbase), lon_(LongitudinalParams{}), steer_(rc.steering) {
  models_[0] = {"차선 (Fast-SCNN)", "WAITING", "", "", 0};
  models_[1] = {"YOLOX-tiny 512x320", "WAITING", "", "", 0};
  apply_settings(Settings{});
}

void ControlLoop::apply_settings(const Settings & s) {
  std::lock_guard<std::mutex> l(m_);
  if (s.memory_mode != s_.memory_mode) clear_memories();   // 방식을 바꾸면 두 기억 모두 새로 시작
  s_ = s;
  auto lp = s.lane_memory;
  lp.front_axle_to_rear_bumper = rc_.wheelbase + s.planner.rear_overhang;
  lp.range_noise_base = s.memory.range_noise_base;
  lp.range_noise_per_m = s.memory.range_noise_per_m;
  lane_mem_.set_params(lp);
  rc_.camera.roi_height_ratio = s.roi_height_percent / 100.0;   // 검출 ROI를 바꾸면 '보여야 할 콘' 판정도 같이
  auto mp = s.memory;
  mp.camera_x = rc_.camera.camera_x;
  mp.front_axle_to_rear_bumper = rc_.wheelbase + s.planner.rear_overhang;
  memory_.set_params(mp);
  auto pp = s.planner;
  pp.wheelbase = rc_.wheelbase;
  pp.max_steer_rad = rc_.steering.max_steer_rad;
  pp.steer_rate_rad_s = s.steering_rate_rad_s;
  planner_.set_params(pp, s.control);
  lon_.set_params(s.lon);
  auto sp = rc_.steering;
  sp.steering_current_weight = s.steering_current_weight;
  sp.rate_limit_rad_s = s.steering_rate_rad_s;
  steer_.set_params(sp);
  sm_.set_link_loss_stop_sec(s.link_loss_stop_sec);
  auto lead = s.lead;
  lead.ego_half_width = s.planner.width / 2;
  lead.front_overhang = s.planner.length - rc_.wheelbase - s.planner.rear_overhang;
  if (s.lon.mode == 2) {   // duty-only: 회생 감속 능력에 맞춰 계획 (전류 제동을 전제로 한 acc.decel보다 크지 않게)
    const auto & d = s.lon.duty_only;
    lead.duty_only = true;
    lead.decel = std::min(lead.decel, d.planning_deceleration_mps2);
    lead.coast_decel = d.coast_deceleration_mps2;
    lead.release_speed = d.coast_release_speed_mps;
    lead.release_delay = d.coast_release_delay_sec;
    lead.response = d.response_sec;
  }
  lead_.set_params(lead);
  auto tp = s.traffic;
  tp.front_overhang = lead.front_overhang;
  if (s.lon.mode == 2) {   // duty-only: 회생 감속으로 줄이고 모터를 풀어 굴러서 정지선 앞에 선다
    const auto & d = s.lon.duty_only;
    tp.duty_only = true;
    tp.decel = std::min(tp.decel, d.planning_deceleration_mps2);
    tp.coast_decel = d.coast_deceleration_mps2;
    tp.release_speed = d.coast_release_speed_mps;
    tp.release_delay = d.coast_release_delay_sec;
    tp.response = d.response_sec;
  }
  traffic_.set_params(tp);
  rc_.lane_timeout_sec = s.lane_timeout_sec;
}

void ControlLoop::on_erpm(int erpm, double t) {
  const double v = erpm_to_speed_mps(erpm, rc_.wheel_diameter_m, rc_.motor_pole_pairs, rc_.motor_pinion_teeth, rc_.spur_gear_teeth,
                                     rc_.differential_pinion_teeth, rc_.differential_ring_teeth, rc_.erpm_direction_sign,
                                     rc_.speed_scale_correction);
  on_speed(v, t);
}

void ControlLoop::on_speed(double v, double t) {
  std::lock_guard<std::mutex> l(m_);
  lon_.on_speed(v, t);
  speed_ = lon_.speed();
  speed_atomic_.store(speed_, std::memory_order_relaxed);
  last_erpm_t_ = t;
  last_erpm_atomic_.store(t, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lo(odom_m_);
  odom_speed_ = speed_;
}

void ControlLoop::on_yaw_rate(double w, double t) {
  std::lock_guard<std::mutex> lo(odom_m_);   // 주 락(m_)을 잡지 않는다
  // 정지 중 영점 (semi_slam과 같은 방식: 시정수 약 2 s). 움직이는 동안은 마지막 값 유지
  if (last_yaw_t_ >= 0 && std::abs(odom_speed_) < 0.02 && t - last_erpm_atomic_.load(std::memory_order_relaxed) < 0.5) {
    const double dt = std::clamp(t - last_yaw_t_, 0.0, 0.05), a = dt / (2.0 + dt);
    const double b = gyro_bias_init_ ? gyro_bias_.load(std::memory_order_relaxed) + a * (w - gyro_bias_.load(std::memory_order_relaxed)) : w;
    gyro_bias_.store(b, std::memory_order_relaxed);
    gyro_bias_init_ = true;
  }
  const double wc = w - gyro_bias_.load(std::memory_order_relaxed);
  last_yaw_rate_.store(wc, std::memory_order_relaxed);
  if (last_yaw_t_ < 0) odom_.reset({0, 0, 0}, t);
  else odom_.update(t, odom_speed_, wc);
  last_yaw_t_ = t;
}

Pose2 ControlLoop::odom_pose() const { std::lock_guard<std::mutex> lo(odom_m_); return odom_.pose(); }

std::optional<Pose2> ControlLoop::odom_pose_at(double t) const { std::lock_guard<std::mutex> lo(odom_m_); return odom_.pose_at(t); }

double ControlLoop::odom_distance() const { std::lock_guard<std::mutex> lo(odom_m_); return odom_.distance(); }

std::optional<double> ControlLoop::odom_distance_at(double t) const { std::lock_guard<std::mutex> lo(odom_m_); return odom_.distance_at(t); }

std::vector<Vec2> ControlLoop::lane_memory_cones(const Pose2 & pose_lane, std::vector<Vec2> * blind_world, double * blind_end) {
  const auto & ref = lane_.center.size() >= 4 ? lane_.center : lane_.path;
  std::vector<Vec2> world;
  lane_mem_view_.clear();
  lane_f_valid_ = false;
  if (ref.size() < 4) return world;
  auto f = frenet_frame(extend_backward(smooth_center(ref), s_.planner.backward_extension));
  const Vec2 origin = project_sl({0.0, 0.0}, f);
  for (auto & a : f.arc) a -= origin.x;
  // 1) 지난 차선 프레임 이후 바퀴로 간 거리만큼 앞뒤 위치를 당긴다 (자이로 미사용)
  const double d_now = odom_distance_at(lane_.capture_time).value_or(odom_distance());
  if (last_lane_dist_ >= 0) {
    const double ds = d_now - last_lane_dist_;
    lane_mem_.advance(ds);
    lock_.advance(ds);
    if (ds > 0) for (auto & q : latest_sl_) q.x -= ds;
  }
  last_lane_dist_ = d_now;
  // 2) 새 검출: 촬영 후 간 거리만큼 앞쪽 좌표를 빼고 현재 차선 기준 (s,l)로
  if (pending_det_) {
    DetectionFrame det = std::move(*pending_det_);
    pending_det_.reset();
    const auto d_cap = odom_distance_at(det.capture_time);
    if (!d_cap) {
      ++dropped_;
    } else {
      if (det.depth_valid) {
        last_det_ = det;   // 깊이 없는 프레임으로 화면의 관측 콘을 지우지 않음
        const double shift = d_now - *d_cap;
        std::vector<Vec2> sl;
        std::vector<double> ranges;
        remember_vehicles(det, d_cap);
        for (const auto & c : obstacle_cells(det.cones, s_.cone_size_m, s_.avoid_vehicles && !s_.lead.enabled)) {
          sl.push_back(project_sl({c.center.x - shift, c.center.y}, f));
          ranges.push_back(std::hypot(c.center.x - rc_.camera.camera_x, c.center.y));
        }
        latest_sl_ = sl;
        latest_det_t_ = det.capture_time;
        lane_mem_.update(sl, ranges);
      }
    }
  }
  lane_f_ = f;
  lane_f_valid_ = true;
  if (s_.memory_mode == 3) {   // 기억 없음: 마지막 검출 프레임에 보인 콘만 (0.3 s 넘게 검출이 없으면 콘 없음)
    if (lane_.capture_time - latest_det_t_ > 0.3) latest_sl_.clear();
    for (const auto & q : latest_sl_) world.push_back(to_world(from_frenet(q.x, q.y, f), pose_lane));   // 화면엔 관측 콘(주황)만: 기억이 아님
    return world;
  }
  // 3) 계획용: (s,l) → 차선 촬영 시점 차량 좌표 → 월드
  for (const auto & tr : lane_mem_.tracks()) {
    const Vec2 local = from_frenet(tr.s, tr.l, f);
    lane_mem_view_.push_back({local.x, local.y, lane_mem_.confirmed(tr), tr.id});
    if (!lane_mem_.confirmed(tr)) continue;
    if (blind_world && tr.s < s_.lane_memory.visible_s_min) {   // 화면을 벗어난 콘: 계획에 넣지 않고 경로 고정 길이만 정함
      blind_world->push_back(to_world(local, pose_lane));
      if (blind_end) *blind_end = std::max(*blind_end, tr.s + rc_.wheelbase + 0.12 + s_.lane_memory.delete_behind);
      continue;
    }
    world.push_back(to_world(local, pose_lane));
  }
  return world;
}

void ControlLoop::on_vesc_connected(bool c, double) { std::lock_guard<std::mutex> l(m_); vesc_connected_ = c; }

void ControlLoop::on_handover_mode(const std::string & mode, double t) { std::lock_guard<std::mutex> l(m_); sm_.on_handover_mode(mode, t); }

void ControlLoop::on_detections(const DetectionFrame & f) { std::lock_guard<std::mutex> l(m_); pending_det_ = f; }

void ControlLoop::set_attitude(const AttitudeInfo & a) { std::lock_guard<std::mutex> l(m_); attitude_ = a; }

void ControlLoop::set_camera_geometry(const CameraGeometry & g) { std::lock_guard<std::mutex> l(m_); rc_.camera = g; }

void ControlLoop::set_model_status(int i, const InspectionModel & m) { std::lock_guard<std::mutex> l(m_); if (i == 0 || i == 1) models_[i] = m; }

void ControlLoop::on_inspection_done(bool ok, const std::string & reason, double t) { std::lock_guard<std::mutex> l(m_); sm_.on_inspection_done(ok, t, reason); }

void ControlLoop::on_refit_done(bool ok, const std::string & reason, double t) {
  std::lock_guard<std::mutex> l(m_);
  sm_.on_refit_done(ok, t, reason);
  clear_memories();   // 자세가 바뀌었으니 콘 기억을 모두 비우고 다시 쌓는다
}

void ControlLoop::clear_memories() {
  memory_.clear();
  lane_mem_.clear();
  lane_mem_view_.clear();
  lock_.clear();
  latest_sl_.clear();
  last_lane_dist_ = -1;
  planner_.reset();
}

Readiness ControlLoop::readiness(double now) const {
  Readiness r;
  r.attitude_ok = attitude_.ok;
  r.lane_model_ready = models_[0].status == "READY";
  r.yolo_model_ready = models_[1].status == "READY";
  r.vesc_connected = vesc_connected_;
  r.erpm_fresh = now - last_erpm_t_ <= rc_.erpm_timeout_sec;
  r.lane_fresh = lane_.valid && (rc_.lane_timeout_sec <= 0 || now - last_lane_t_ <= rc_.lane_timeout_sec);   // 0 = 제한 없음
  r.client_alive = sm_.client_alive(now);
  return r;
}

void ControlLoop::integrate_detections() {
  if (!pending_det_) return;
  DetectionFrame f = std::move(*pending_det_);
  pending_det_.reset();
  // 위치는 깊이 영상으로 계산하므로 차 자세도 깊이 촬영 시각으로 (RGB 시각과 최대 한 프레임 다름 → 회전 중 옆으로 어긋남)
  const double t_pose = (f.depth_time > 0 ? f.depth_time : f.capture_time) + s_.obs_time_offset_s;
  auto pose = odom_pose_at(t_pose);
  if (!pose) pose = odom_pose_at(f.capture_time);
  if (!pose) { ++dropped_; return; }   // 오도메트리 이력 밖(너무 오래됨·미래) → 위치를 믿을 수 없음
  if (!f.depth_valid) return;          // 깊이 없는 프레임은 관측으로 치지 않음 (기억 콘을 놓침으로 세지 않음)
  last_det_ = f;
  const auto cones = obstacle_cells(f.cones, s_.cone_size_m, s_.avoid_vehicles && !s_.lead.enabled);
  remember_vehicles(f, odom_distance_at(f.capture_time));
  const auto cam = rc_.camera;
  const double cone = s_.cone_size_m;
  std::vector<std::uint32_t> ids;
  memory_.update(*pose, f.capture_time, cones, [cam, cone](const std::vector<Vec2> & centers, const Pose2 & p) {
    std::vector<Vec2> local;
    local.reserve(centers.size());
    for (const auto & w : centers) local.push_back(to_local(w, p));
    return expected_visible(local, cam, cone);
  }, on_cone_log ? &ids : nullptr);
  if (on_cone_log && !cones.empty() && ids.size() == cones.size()) {
    std::vector<ConeLogRow> rows;
    rows.reserve(cones.size());
    for (std::size_t i = 0; i < cones.size(); ++i) {
      const auto & c = cones[i];
      ConeLogRow r;
      r.t_rgb = f.capture_time; r.t_depth = f.depth_time; r.t_pose = t_pose;
      r.id = ids[i]; r.class_id = c.class_id;
      r.local_x = c.center.x; r.local_y = c.center.y; r.front_x = c.front.x; r.front_y = c.front.y; r.range = c.range; r.bearing = c.bearing;
      r.pose_x = pose->x; r.pose_y = pose->y; r.pose_yaw = pose->yaw;
      r.speed = speed_; r.yaw_rate = last_yaw_rate_.load(std::memory_order_relaxed); r.gyro_bias = gyro_bias_.load(std::memory_order_relaxed);
      const Vec2 w = to_world(c.center, *pose);
      r.world_x = w.x; r.world_y = w.y;
      if (const auto * tr = memory_.find(ids[i])) { r.mem_x = tr->world.x; r.mem_y = tr->world.y; r.hits = tr->hits; }
      rows.push_back(r);
    }
    on_cone_log(rows);
  }
}

void ControlLoop::on_traffic_signal(TrafficSignal signal, double t) {
  std::lock_guard<std::mutex> l(m_);
  traffic_.on_signal(signal, t);
}

void ControlLoop::remember_vehicles(const DetectionFrame & det, std::optional<double> dist) {
  if (!det.depth_valid || !dist) return;
  new_vehicles_.clear();
  for (const auto & c : det.cones) if (c.class_id == 1 || c.class_id == 2) new_vehicles_.push_back(c);
  new_vehicles_dist_ = *dist;
  new_det_ = true;
}

StepOutput ControlLoop::on_lane(const LaneObservation & lane, double now) {
  std::lock_guard<std::mutex> l(m_);
  const double t0 = now_sec();
  if (s_.memory_mode == 0) integrate_detections();
  if (lane.valid) { lane_ = lane; last_lane_t_ = now; }
  bool have_path = false;
  if (lane_.valid) {
    const auto pose_lane = odom_pose_at(lane_.capture_time).value_or(odom_pose());
    const bool commit = s_.memory_mode == 2;
    std::vector<Vec2> blind;
    double blind_end = -1;
    const auto cones = s_.memory_mode >= 1 ? lane_memory_cones(pose_lane, commit ? &blind : nullptr, &blind_end) : memory_.confirmed_world();
    rec_cones_ = cones;
    if (commit) lock_.refresh(blind_end);
    plan_ = planner_.plan(lane_, cones, pose_lane, speed_, steer_.angle(), commit ? lock_.pin() : nullptr, commit ? &blind : nullptr);
    if (commit && lane_f_valid_) {   // 다음 고정에 쓸 이번 경로 (s, l)
      std::vector<Vec2> sl;
      sl.reserve(plan_.path_local.size());
      for (const auto & q : plan_.path_local) sl.push_back(project_sl(q, lane_f_));
      lock_.set_last_plan(std::move(sl));
    }
    plan_pose_ = pose_lane;
    {   // 차량 추종: 새 검출 차량을 경로 시각으로 옮겨(바퀴 이동 거리만큼 앞쪽을 뺌) 앞차 갱신
      const double d_lane = odom_distance_at(lane_.capture_time).value_or(odom_distance());
      std::vector<ConeObservation> vehicles;
      if (new_det_) {
        const double shift = d_lane - new_vehicles_dist_;
        for (auto v : new_vehicles_) { v.center.x -= shift; v.front.x -= shift; vehicles.push_back(v); }
      }
      lead_.update(plan_.path_local, vehicles, new_det_, lane_.capture_time, d_lane, speed_);
      new_det_ = false;
      // 신호등 정지: 이번에 새로 들어온 차선 결과의 정지선만 관측으로 (재사용한 lane_은 NaN)
      traffic_.update(lane.valid ? lane.stop_line_m : std::numeric_limits<double>::quiet_NaN(), lane_.capture_time, d_lane, speed_);
    }
    // 계획 자세 → 현재 자세로 옮겨 제어
    const Pose2 now_pose = odom_pose();
    control_path_.clear();
    for (const auto & q : plan_.path_local) control_path_.push_back(to_local(to_world(q, pose_lane), now_pose));
    have_path = control_path_.size() >= 2;
  }
  stats_.record("memory_plan", t0, now_sec());
  if (on_record && lane.valid) {
    RecordFrame r;
    r.t = lane_.capture_time;
    r.pose = plan_pose_;
    r.speed = speed_;
    r.yaw_rate = last_yaw_rate_.load(std::memory_order_relaxed);
    r.state = state_name(sm_.state());
    r.lane_valid = lane.valid;
    r.left = lane_.left; r.right = lane_.right; r.path = lane_.path; r.center = lane_.center;
    r.plan_mode = plan_mode_name(plan_.mode);
    r.rollout_ok = plan_.rollout_ok;
    r.obstacle_active = plan_.obstacle_active;
    r.plan_path = plan_.path_local;
    r.cones_world = rec_cones_;
    on_record(r);
  }
  return control_step(now, have_path);
}

StepOutput ControlLoop::tick(double now) {
  std::lock_guard<std::mutex> l(m_);
  // 차선 결과가 잠깐 늦어도(lane_timeout 안) 마지막 계획 경로를 현재 자세로 옮겨 계속 추종한다. 그 이상은 상태기계가 정지
  bool have_path = false;
  if (lane_.valid && (rc_.lane_timeout_sec <= 0 || now - last_lane_t_ <= rc_.lane_timeout_sec) && !plan_.path_local.empty()) {
    const Pose2 now_pose = odom_pose();
    control_path_.clear();
    for (const auto & q : plan_.path_local) control_path_.push_back(to_local(to_world(q, plan_pose_), now_pose));
    have_path = control_path_.size() >= 2;
  }
  return control_step(now, have_path);
}

StepOutput ControlLoop::control_step(double now, bool have_path) {
  const double t0 = now_sec();
  const Readiness r = readiness(now);
  sm_.tick(r, speed_, now);
  const double dt = last_control_t_ < 0 ? 0.01 : std::clamp(now - last_control_t_, 1e-4, 0.1);
  last_control_t_ = now;
  StepOutput out;
  out.servo = steer_.servo(steer_.angle());
  const bool stop = sm_.state() != DriveState::Driving;
  std::optional<OrderedPath> path;
  if (have_path) {
    std::vector<Point> pts;
    pts.reserve(control_path_.size());
    for (const auto & q : control_path_) pts.push_back({q.x, q.y});
    path = build_ordered_path(pts, 4, rc_.path_min_span_m, -0.5, 3.5, rc_.path_max_gap_m, rc_.geometry_window_m);
  }
  double plan_target = 0.0;
  if (path) {
    try {
      const double plan_decel = s_.lon.mode == 2 ? s_.lon.duty_only.planning_deceleration_mps2 : s_.planning_decel;   // duty-only는 회생 감속 능력으로
      const auto sp = plan_path_speeds(*path, s_.lon.minimum_speed_mps, s_.lon.maximum_speed_mps, s_.maximum_lateral_accel,
                                       plan_decel, s_.lon.planning_acceleration_mps2, s_.lon.maximum_speed_mps, 0.0, 0.08);
      plan_target = sp.target_nominal_speed_mps;
    } catch (const std::exception &) {
      path.reset();
    }
  }
  if (path) {
    const auto st = stanley_control(*path, speed_, s_.stanley_gain, s_.stanley_softening, s_.heading_lookahead_m,
                                    rc_.steering.max_steer_rad, rc_.steering.corner_heading_threshold_rad, 0.8);
    cte_ = st.cross_track_error_m;
    // 정지 시 조향 유지는 '목표 속도 0'일 때만 걸린다 → 차량 추종·롤아웃 상한까지 반영한 실제 목표를 넘긴다
    // (곡률 계획 속도만 넘기면 앞차 뒤에 서 있어도 목표>0이라 차선 인식 흔들림을 그대로 따라 조향이 떨린다)
    const double cap_now = std::min({plan_.speed_cap, s_.lead.enabled ? lead_.state().cap : plan_.speed_cap, traffic_.state().cap});
    steer_.update(st.steering_angle_rad, dt, st.heading_error_rad, stop ? 0.0 : std::min(plan_target, cap_now), speed_);
  }
  const double cap = std::min({plan_.speed_cap, s_.lead.enabled ? lead_.state().cap : plan_.speed_cap, traffic_.state().cap});   // 차량 추종·신호등 상한
  const auto cmd = lon_.update(dt, path ? plan_target : 0.0, plan_.obstacle_active, cap, stop || !path);
  target_ = cmd.target_speed;
  out.duty = cmd.duty;
  out.brake_current = cmd.brake_current;
  out.brake_mode = cmd.brake_mode;
  out.servo = steer_.servo(steer_.angle());
  out.publish = sm_.auto_output_allowed();
  if (!out.publish && sm_.state() != DriveState::Stopping) lon_.reset();
  if (now - last_request_t_ >= rc_.handover_request_period_sec) {
    if (sm_.wants_auto_rearm()) out.handover_request = "auto";
    else if (sm_.wants_manual_handover()) out.handover_request = "manual";
    if (!out.handover_request.empty()) last_request_t_ = now;
  }
  last_out_ = out;
  stats_.record("control", t0, now_sec());
  return out;
}

std::pair<bool, std::string> ControlLoop::request_start(double now) {
  std::lock_guard<std::mutex> l(m_);
  auto res = sm_.request_start(readiness(now), now);
  if (res.first) { lon_.reset(); steer_.reset(); last_request_t_ = -1e9; }
  return res;
}
void ControlLoop::request_stop(const std::string & reason, double now) { std::lock_guard<std::mutex> l(m_); sm_.request_stop(reason, now); }
std::pair<bool, std::string> ControlLoop::request_refit(double now) { std::lock_guard<std::mutex> l(m_); return sm_.request_refit(now); }
std::pair<bool, std::string> ControlLoop::request_mode(DriveMode m, double now) { std::lock_guard<std::mutex> l(m_); return sm_.request_mode(m, now); }
void ControlLoop::on_heartbeat(const std::string & c, double now) { std::lock_guard<std::mutex> l(m_); sm_.on_heartbeat(c, now); }
bool ControlLoop::refit_pending() const { std::lock_guard<std::mutex> l(m_); return sm_.state() == DriveState::Refitting; }
DriveState ControlLoop::state() const { std::lock_guard<std::mutex> l(m_); return sm_.state(); }

bool ControlLoop::cones_nearby() const {
  std::lock_guard<std::mutex> l(m_);
  if (s_.memory_mode >= 1) {
    for (const auto & m : lane_mem_view_) if (m.x > -0.6 && m.x < 3.5 && std::abs(m.y) < 1.0) return true;
    return !last_det_.cones.empty();
  }
  const Pose2 p = odom_pose();
  for (const auto & w : memory_.confirmed_world()) {
    const Vec2 q = to_local(w, p);
    if (q.x > -0.6 && q.x < 3.5 && std::abs(q.y) < 1.0) return true;
  }
  return false;
}

TelemetryFrame ControlLoop::telemetry(bool full, double now) const {
  std::lock_guard<std::mutex> l(m_);
  TelemetryFrame f;
  f.full = full;
  f.t = now;
  f.state = state_name(sm_.state());
  f.mode = mode_name(sm_.mode());
  f.reason = sm_.last_reason();
  f.readiness = readiness(now);
  f.inspection.attitude_status = attitude_.status;
  f.inspection.attitude_message = attitude_.message;
  f.inspection.height_m = attitude_.height_m;
  f.inspection.roll_deg = attitude_.roll_deg;
  f.inspection.pitch_down_deg = attitude_.pitch_down_deg;
  f.inspection.models = {models_[0], models_[1]};
  if (!full) return f;
  f.vehicle = {speed_, target_, steer_.angle() * kRad2Deg, cte_, last_out_.duty, last_out_.brake_current};
  // 화면은 현재 자세 기준 (차선은 촬영 시점 기준이지만 지연이 작아 그대로 표시)
  f.lane_left = lane_.left;
  f.lane_right = lane_.right;
  f.lane_center = lane_.center;
  f.lane_path = lane_.path;
  f.plan_path = plan_.mode == PlanMode::Center ? std::vector<Vec2>{} : control_path_;
  f.plan_mode = plan_mode_name(plan_.mode);
  f.rollout_ok = plan_.rollout_ok;
  static const char * kNames[] = {"con", "truck", "sedan", "traffic_light"};
  for (const auto & c : last_det_.cones)
    f.detections.push_back({kNames[std::clamp(c.class_id, 0, 3)], c.score, c.center.x, c.center.y, c.length, c.width, c.bearing});
  const Pose2 p = odom_pose();
  if (s_.memory_mode >= 1) {
    f.memory = lane_mem_view_;   // 차선 촬영 시점 기준 (지연이 작아 그대로 표시)
  } else {
    for (const auto & tr : memory_.tracks()) {
      const Vec2 q = to_local(tr.world, p);
      f.memory.push_back({q.x, q.y, memory_.confirmed(tr), tr.id});
    }
  }
  f.memory_mode = s_.memory_mode;
  f.lock_m = lock_.remaining();
  f.lead = lead_.state();
  f.traffic = traffic_.state();
  f.traffic_enabled = s_.traffic.enabled;
  f.lon_mode = lon_.active_mode();
  f.lon_mode_requested = s_.lon.mode;
  f.lon_phase = lon_.phase();
  f.stages = stats_.snapshot(now_sec());
  return f;
}

}  // namespace auto_control::drive
