#include "cone_drive_test/core/control_loop.hpp"
#include "cone_drive_test/core/cone_memory.hpp"
#include "cone_drive_test/core/visibility.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
}

ControlLoop::ControlLoop(DriveMode mode, const RuntimeConfig & rc)
  : rc_(rc), sm_(mode, 1.0), memory_(MemoryParams{}), planner_(PlannerParams{}, ControlParams{}), odom_(rc.wheelbase), lon_(LongitudinalParams{}), steer_(rc.steering) {
  models_[0] = {"차선 (Fast-SCNN)", "WAITING", "", "", 0};
  models_[1] = {"YOLOX-tiny 512x320", "WAITING", "", "", 0};
  apply_settings(Settings{});
}

void ControlLoop::apply_settings(const Settings & s) {
  std::lock_guard<std::mutex> l(m_);
  if (s.memory_mode != s_.memory_mode) {   // 방식을 바꾸면 두 기억 모두 새로 시작
    memory_.clear();
    lane_mem_.clear();
    lane_mem_view_.clear();
    lock_.clear();
    latest_sl_.clear();
    last_lane_dist_ = -1;
    planner_.reset();
  }
  s_ = s;
  auto lp = s.lane_memory;
  lp.front_axle_to_rear_bumper = rc_.wheelbase + 0.12;
  lp.range_noise_base = s.memory.range_noise_base;
  lp.range_noise_per_m = s.memory.range_noise_per_m;
  lane_mem_.set_params(lp);
  rc_.camera.roi_height_ratio = s.roi_height_percent / 100.0;   // 검출 ROI를 바꾸면 '보여야 할 콘' 판정도 같이
  auto mp = s.memory;
  mp.camera_x = rc_.camera.camera_x;
  mp.front_axle_to_rear_bumper = rc_.wheelbase + 0.12;
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
  std::lock_guard<std::mutex> lo(odom_m_);
  odom_speed_ = speed_;
}

void ControlLoop::on_yaw_rate(double w, double t) {
  std::lock_guard<std::mutex> lo(odom_m_);   // 주 락(m_)을 잡지 않는다
  if (last_yaw_t_ < 0) odom_.reset({0, 0, 0}, t);
  else odom_.update(t, odom_speed_, w);
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
      last_det_ = det;
      if (det.depth_valid) {
        const double shift = d_now - *d_cap;
        std::vector<Vec2> sl;
        std::vector<double> ranges;
        for (const auto & c : det.cones) {
          if (c.class_id != 0) continue;
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
    std::uint32_t id = 1;
    for (const auto & q : latest_sl_) {
      const Vec2 local = from_frenet(q.x, q.y, f);
      lane_mem_view_.push_back({local.x, local.y, true, id++});
      world.push_back(to_world(local, pose_lane));
    }
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
  memory_.clear();   // 자세가 바뀌었으니 기억을 다시 쌓는다
  planner_.reset();
}

Readiness ControlLoop::readiness(double now) const {
  Readiness r;
  r.attitude_ok = attitude_.ok;
  r.lane_model_ready = models_[0].status == "READY";
  r.yolo_model_ready = models_[1].status == "READY";
  r.vesc_connected = vesc_connected_;
  r.erpm_fresh = now - last_erpm_t_ <= rc_.erpm_timeout_sec;
  r.lane_fresh = lane_.valid && now - last_lane_t_ <= rc_.lane_timeout_sec;
  r.client_alive = sm_.client_alive(now);
  return r;
}

void ControlLoop::integrate_detections() {
  if (!pending_det_) return;
  DetectionFrame f = std::move(*pending_det_);
  pending_det_.reset();
  const auto pose = odom_pose_at(f.capture_time);
  if (!pose) { ++dropped_; return; }   // 오도메트리 이력 밖(너무 오래됨·미래) → 위치를 믿을 수 없음
  last_det_ = f;
  if (!f.depth_valid) return;          // 깊이 없는 프레임은 관측으로 치지 않음 (기억 콘을 놓침으로 세지 않음)
  std::vector<ConeObservation> cones;
  for (const auto & c : f.cones) if (c.class_id == 0) cones.push_back(c);
  const auto cam = rc_.camera;
  const double cone = s_.cone_size_m;
  memory_.update(*pose, f.capture_time, cones, [cam, cone](const std::vector<Vec2> & centers, const Pose2 & p) {
    std::vector<Vec2> local;
    local.reserve(centers.size());
    for (const auto & w : centers) local.push_back(to_local(w, p));
    return expected_visible(local, cam, cone);
  });
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
    if (commit) lock_.refresh(blind_end);
    plan_ = planner_.plan(lane_, cones, pose_lane, speed_, steer_.angle(), commit ? lock_.pin() : nullptr, commit ? &blind : nullptr);
    if (commit && lane_f_valid_) {   // 다음 고정에 쓸 이번 경로 (s, l)
      std::vector<Vec2> sl;
      sl.reserve(plan_.path_local.size());
      for (const auto & q : plan_.path_local) sl.push_back(project_sl(q, lane_f_));
      lock_.set_last_plan(std::move(sl));
    }
    plan_pose_ = pose_lane;
    // 계획 자세 → 현재 자세로 옮겨 제어
    const Pose2 now_pose = odom_pose();
    control_path_.clear();
    for (const auto & q : plan_.path_local) control_path_.push_back(to_local(to_world(q, pose_lane), now_pose));
    have_path = control_path_.size() >= 2;
  }
  stats_.record("memory_plan", t0, now_sec());
  return control_step(now, have_path);
}

StepOutput ControlLoop::tick(double now) {
  std::lock_guard<std::mutex> l(m_);
  // 차선 결과가 잠깐 늦어도(lane_timeout 안) 마지막 계획 경로를 현재 자세로 옮겨 계속 추종한다. 그 이상은 상태기계가 정지
  bool have_path = false;
  if (lane_.valid && now - last_lane_t_ <= rc_.lane_timeout_sec && !plan_.path_local.empty()) {
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
      const auto sp = plan_path_speeds(*path, s_.lon.minimum_speed_mps, s_.lon.maximum_speed_mps, s_.maximum_lateral_accel,
                                       s_.planning_decel, s_.lon.planning_acceleration_mps2, s_.lon.maximum_speed_mps, 0.0, 0.08);
      plan_target = sp.target_nominal_speed_mps;
    } catch (const std::exception &) {
      path.reset();
    }
  }
  if (path) {
    const auto st = stanley_control(*path, speed_, s_.stanley_gain, s_.stanley_softening, s_.heading_lookahead_m,
                                    rc_.steering.max_steer_rad, rc_.steering.corner_heading_threshold_rad, 0.8);
    cte_ = st.cross_track_error_m;
    steer_.update(st.steering_angle_rad, dt, st.heading_error_rad, stop ? 0.0 : plan_target, speed_);
  }
  const auto cmd = lon_.update(dt, path ? plan_target : 0.0, plan_.obstacle_active, plan_.speed_cap, stop || !path);
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
  f.stages = stats_.snapshot(now_sec());
  return f;
}

}  // namespace cone_drive_test
