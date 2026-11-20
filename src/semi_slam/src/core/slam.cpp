#include "semi_slam/core/slam.hpp"
#include "semi_slam/core/json_writer.hpp"
#include "semi_slam/core/lane_fit.hpp"
#include "semi_slam/core/mini_json.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace semi_slam {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kCap = 0.15;          // 차선 점 오차 상한 (참고 RMS용)
double wrap(double a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }
double dot(const Vec2 & a, const Vec2 & b) { return a.x * b.x + a.y * b.y; }
Vec2 to_world(const Pose & p, const Vec2 & q) {
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y};
}
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// 정지선 S에서 자유 적분 자세를 다시 잡는다: 앞뒤 = S, (BEV 값이 있으면) 옆 = 두 선 가운데 기준, 방향 = 차선 방향 기준
void anchor_at_stop(Pose & p, const Vec2 & rel, const TrackMap & map, bool cw) {
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S, cw), left{-dir.y, dir.x};
  const double along = (p.x - S.x) * dir.x + (p.y - S.y) * dir.y;
  p.x -= along * dir.x;
  p.y -= along * dir.y;
  if (!std::isfinite(rel.x)) return;
  const double lat = -rel.y;                  // 두 선 가운데가 차 기준 왼쪽이면 차는 가운데보다 오른쪽
  p.x = S.x + left.x * lat;
  p.y = S.y + left.y * lat;
  p.yaw = std::atan2(dir.y, dir.x) - rel.x;   // 차선이 차 기준 왼쪽으로 기울어 보이면 차는 오른쪽으로 틀어진 것
}
}  // namespace

// ---------------------------------------------------------------- 보정 값
int OdomCalib::speed_key(double v) { return int(std::lround(std::abs(v) * 20.0)) * 5; }

double OdomCalib::wheel_scale_for(double v) const {
  if (wheel_scale.empty()) return 1.0;
  const double cm = std::abs(v) * 100.0;
  auto hi = wheel_scale.lower_bound(int(std::ceil(cm)));
  if (hi == wheel_scale.begin()) return hi->second;
  if (hi == wheel_scale.end()) return std::prev(hi)->second;
  auto lo = std::prev(hi);
  const double a = (cm - lo->first) / double(hi->first - lo->first);
  return lo->second + std::clamp(a, 0.0, 1.0) * (hi->second - lo->second);
}

void OdomCalib::set_wheel_scale(double v, double s) { wheel_scale[speed_key(v)] = s; }

std::string OdomCalib::to_json() const {
  JsonWriter w;
  w.begin_object().key("version").value((long long)version).key("gyro_scale").value(gyro_scale, 6).key("gyro_laps").value((long long)gyro_laps)
   .key("gyro_bias").value(gyro_bias, 6).key("latency").value(latency, 4);
  w.key("wheel_scale").begin_object();
  for (const auto & [k, s] : wheel_scale) w.key(std::to_string(k).c_str()).value(s, 6);
  w.end_object();
  w.key("wheel_laps").begin_object();
  for (const auto & [k, n] : wheel_laps) w.key(std::to_string(k).c_str()).value((long long)n);
  w.end_object().end_object();
  return w.str();
}

OdomCalib OdomCalib::from_json(const std::string & text) {
  OdomCalib c;
  const auto j = parse_json(text);
  if (!j || j->kind != JsonValue::Object) return c;
  auto num = [&](const char * k, double & out) { const auto * v = j->get(k); if (v && v->kind == JsonValue::Number) out = v->num; };
  double ver = 1, gl = 0;
  num("version", ver);
  c.version = int(ver);
  num("gyro_scale", c.gyro_scale);
  num("gyro_laps", gl);
  c.gyro_laps = int(gl);
  num("gyro_bias", c.gyro_bias);
  num("latency", c.latency);
  if (const auto * ws = j->get("wheel_scale"); ws && ws->kind == JsonValue::Object)
    for (const auto & [k, v] : ws->obj) if (v->kind == JsonValue::Number) c.wheel_scale[std::stoi(k)] = v->num;
  if (const auto * wl = j->get("wheel_laps"); wl && wl->kind == JsonValue::Object)
    for (const auto & [k, v] : wl->obj) if (v->kind == JsonValue::Number) c.wheel_laps[std::stoi(k)] = int(v->num);
  return c;
}

// ---------------------------------------------------------------- 적분 (자유 적분)
Pose Trajectory::at(double t) const {
  if (poses.empty()) return {};
  if (t <= poses.front().t) return poses.front();
  if (t >= poses.back().t) return poses.back();
  const auto it = std::lower_bound(poses.begin(), poses.end(), t, [](const Pose & p, double v) { return p.t < v; });
  const Pose & b = *it, & a = *std::prev(it);
  const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
  return {t, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.yaw + u * wrap(b.yaw - a.yaw), a.dist + u * (b.dist - a.dist)};
}

Trajectory integrate(const RunData & run, const OdomCalib & calib, const TrackMap & map, bool correction) {
  Trajectory tr;
  tr.poses.reserve(run.gyro.size() + 1);
  Pose p = run.start;
  p.dist = 0;
  if (std::isfinite(run.start_rel.x)) anchor_at_stop(p, run.start_rel, map, run.cw);
  tr.poses.push_back(p);
  std::size_t si = 0, ci = 0;
  double v_raw = 0, last_t = p.t;
  const double shift = calib.latency - run.crossing_latency;
  auto advance_speed = [&](double t) { while (si < run.speed.size() && run.speed[si].t <= t) v_raw = run.speed[si++].v; };
  auto step = [&](double t, double w_raw) {
    advance_speed(t);
    const double dt = std::clamp(t - last_t, 0.0, 0.05);
    last_t = t;
    const double v = v_raw * calib.wheel_scale_for(v_raw);
    const double w = calib.gyro_scale * (w_raw - run.gyro_bias0 - calib.gyro_bias);
    const double ym = p.yaw + 0.5 * w * dt;
    p.x += v * std::cos(ym) * dt;
    p.y += v * std::sin(ym) * dt;
    p.yaw = wrap(p.yaw + w * dt);
    p.dist += v * dt;
    p.t = t;
    while (ci < run.crossing_t.size() && run.crossing_t[ci] + shift <= t) {
      if (correction) anchor_at_stop(p, ci < run.anchor_rel.size() ? run.anchor_rel[ci] : Vec2{kNaN, 0}, map, run.cw);
      ++ci;
    }
    tr.poses.push_back(p);
  };
  if (!run.gyro.empty()) {
    for (const auto & g : run.gyro) if (g.t >= run.start.t) step(g.t, g.w);
  } else {
    for (const auto & s : run.speed) if (s.t >= run.start.t) step(s.t, 0.0);
  }
  return tr;
}

// ---------------------------------------------------------------- 평가 (참고)
Evaluation evaluate(const RunData & run, const OdomCalib & calib, const TrackMap & map, double max_range, double cap) {
  Evaluation e;
  const auto tr = integrate(run, calib, map, run.stopline_correction);
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S, run.cw);
  const double Lc = map.center_length();
  const std::size_t nl = run.crossing_t.size() + 1;
  std::vector<double> cross = run.crossing_t;
  for (auto & c : cross) c += calib.latency - run.crossing_latency;
  std::vector<double> lap_sq(nl, 0.0);
  std::vector<int> lap_n(nl, 0);
  double sq = 0, ssq = 0;
  int ns = 0;
  for (std::size_t k = 0; k < run.lanes.size(); k += 2) {
    const auto & L = run.lanes[k];
    const Pose p = tr.at(L.t + calib.latency);
    const std::size_t lap = std::size_t(std::upper_bound(cross.begin(), cross.end(), L.t + calib.latency) - cross.begin());
    for (int side = 0; side < 2; ++side)
      for (const auto & q : side == 0 ? L.left : L.right) {
        if (q.x < 0 || q.x > max_range) continue;
        const Vec2 w = to_world(p, q);
        const double d = std::min(cap, map.dist_side(side, run.cw, w));
        sq += d * d; lap_sq[lap] += d * d; ++lap_n[lap]; ++e.points;
      }
    if (std::isfinite(L.stop_m) && L.stop_m > 0.15 && L.stop_m < max_range) {
      const Vec2 w = to_world(p, {L.stop_m, 0.0});
      if (std::hypot(w.x - S.x, w.y - S.y) < 0.7) { const double a = dot({w.x - S.x, w.y - S.y}, dir); ssq += a * a; ++ns; }
    }
  }
  e.rms_m = e.points ? std::sqrt(sq / e.points) : kNaN;
  e.stop_rms_m = ns ? std::sqrt(ssq / ns) : 0.0;
  e.accuracy = e.points ? std::max(0.0, 1.0 - e.rms_m / (0.5 * map.lane_width())) : 0.0;
  e.cost = (e.points ? sq / e.points : 1.0) + (ns ? ssq / ns : 0.0);
  double prev_d = 0, prev_t = run.start.t;
  std::size_t gi = 0;
  for (std::size_t k = 0; k < run.crossing_t.size(); ++k) {
    const Pose p = tr.at(cross[k] - 1e-3);
    LapMetrics m;
    m.lap = int(k) + 1;
    m.along_err_m = dot({p.x - S.x, p.y - S.y}, dir);
    m.heading_err_deg = wrap(p.yaw - std::atan2(dir.y, dir.x)) * 180.0 / kPi;
    m.dist_m = p.dist - prev_d;
    m.s_err_m = m.dist_m - Lc;
    m.length_err = m.dist_m / std::max(1e-6, Lc) - 1.0;
    double turn = 0, gt = prev_t;
    while (gi < run.gyro.size() && run.gyro[gi].t <= run.crossing_t[k]) {
      const auto & g = run.gyro[gi++];
      if (g.t <= prev_t) continue;
      turn += calib.gyro_scale * (g.w - run.gyro_bias0 - calib.gyro_bias) * std::clamp(g.t - gt, 0.0, 0.05);
      gt = g.t;
    }
    m.turn_deg = (run.cw ? -turn : turn) * 180.0 / kPi;
    prev_d = p.dist;
    prev_t = run.crossing_t[k];
    m.points = lap_n[k];
    m.rms_m = lap_n[k] ? std::sqrt(lap_sq[k] / lap_n[k]) : kNaN;
    e.laps.push_back(m);
  }
  return e;
}

// ---------------------------------------------------------------- 보정 (S → S 기준)
OdomEstimate estimate_odometry(const RunData & run, const TrackMap & map) {
  OdomEstimate e;
  const int N = int(run.crossing_t.size());
  e.laps = N;
  e.ref_length = map.center_length();
  if (N < 2) { e.message = "바퀴가 2바퀴 미만이라 보정하지 않음"; return e; }
  const double t0 = run.start.t, t1 = run.crossing_t.back();
  // 자이로: 원값 회전량 (출발 직전 정지 영점만 뺌)
  double raw_turn = 0, lt = t0;
  for (const auto & g : run.gyro) {
    if (g.t <= t0) continue;
    if (g.t > t1) break;
    raw_turn += (g.w - run.gyro_bias0) * std::clamp(g.t - lt, 0.0, 0.05);
    lt = g.t;
  }
  double true_turn = (run.cw ? -2.0 : 2.0) * kPi * N;
  // 처음·마지막 S에서 차가 차선 대비 비스듬했던 만큼 (차 방향 = 차선 방향 − rel)
  const Vec2 last_rel = std::size_t(N) <= run.anchor_rel.size() ? run.anchor_rel[std::size_t(N) - 1] : Vec2{kNaN, 0};
  if (std::isfinite(run.start_rel.x) && std::isfinite(last_rel.x)) true_turn -= last_rel.x - run.start_rel.x;
  // 바퀴: 원값 거리
  double raw_dist = 0, v_cur = 0, st = t0;
  for (const auto & s : run.speed) {
    if (s.t <= t0) { v_cur = s.v; continue; }
    const double tt = std::min(s.t, t1);
    raw_dist += std::abs(v_cur) * std::max(0.0, tt - st);
    st = tt;
    v_cur = s.v;
    if (s.t >= t1) break;
  }
  if (st < t1) raw_dist += std::abs(v_cur) * (t1 - st);
  if (std::abs(raw_turn) < 1e-6 || raw_dist < 1e-3) { e.message = "자이로·속도 기록이 없음"; return e; }
  e.gyro_scale = true_turn / raw_turn;
  e.wheel_scale = N * e.ref_length / raw_dist;
  e.raw_turn_deg_per_lap = std::abs(raw_turn) / N * 180.0 / kPi;
  e.raw_dist_per_lap = raw_dist / N;
  if (e.gyro_scale < 0.8 || e.gyro_scale > 1.2 || e.wheel_scale < 0.8 || e.wheel_scale > 1.2) {
    e.message = "계산한 배율이 범위(0.8~1.2) 밖: 바퀴 세기·기록을 확인하세요";
    return e;
  }
  e.ok = true;

  // 주행 중 검산: 직선(요레이트 작음)에서 BEV 차선 각도 vs 옆으로 밀리는 속도
  struct F { double t, ang, cy; bool straight; };
  std::vector<F> fr;
  std::size_t gi = 0;
  for (const auto & L : run.lanes) {
    while (gi + 1 < run.gyro.size() && run.gyro[gi + 1].t <= L.t) ++gi;
    if (run.gyro.empty()) break;
    const double w = run.gyro[gi].w - run.gyro_bias0;
    const auto f = fit_lane_frame(L.left, L.right, 0.15, 0.8, 0.3);
    if (!f.both) continue;
    fr.push_back({L.t, f.angle, f.center_y, std::abs(w) < 0.1});
  }
  double wsum = 0, ang_sum = 0, slip_sum = 0;
  const double v = std::max(0.1, e.raw_dist_per_lap * N / std::max(1e-6, t1 - t0) * e.wheel_scale);
  std::size_t i = 0;
  while (i < fr.size()) {
    if (!fr[i].straight) { ++i; continue; }
    std::size_t j = i;
    while (j + 1 < fr.size() && fr[j + 1].straight && fr[j + 1].t - fr[j].t < 0.3) ++j;
    const double dur = fr[j].t - fr[i].t;
    if (dur > 0.4 && j - i >= 10) {
      double n = 0, st2 = 0, sy = 0, stt = 0, sty = 0, sa = 0;
      for (std::size_t k = i; k <= j; ++k) {
        const double t = fr[k].t - fr[i].t;
        ++n; st2 += t; sy += fr[k].cy; stt += t * t; sty += t * fr[k].cy; sa += fr[k].ang;
      }
      const double rate = (n * sty - st2 * sy) / std::max(1e-9, n * stt - st2 * st2);   // 차선 가운데가 차 기준 왼쪽으로 움직이는 속도
      const double slip = std::asin(std::clamp(rate / v, -0.5, 0.5));   // 차가 차선보다 오른쪽을 향한 각 (실제)
      ang_sum += sa / n * dur; slip_sum += slip * dur; wsum += dur;
      ++e.straights;
    }
    i = j + 1;
  }
  if (wsum > 0) {
    e.straight_angle_deg = ang_sum / wsum * 180.0 / kPi;
    e.straight_slip_deg = slip_sum / wsum * 180.0 / kPi;
    e.bev_yaw_residual_deg = e.straight_angle_deg - e.straight_slip_deg;
  }
  return e;
}

CalibResult calibrate(const RunData & run, const OdomCalib & current, const TrackMap & map, double max_range, int max_prev_laps,
                      double outlier_ratio) {
  CalibResult r;
  r.calib = current;
  r.calib.version = OdomCalib::kVersion;
  r.est = estimate_odometry(run, map);
  r.before = evaluate(run, current, map, max_range);
  if (!r.est.ok) { r.after = r.before; r.message = r.est.message; return r; }
  const int N = r.est.laps;
  std::string notes;
  auto merge = [&](double old_v, int old_n, double est, double * out, int * out_n, const char * name) {
    const int n_prev = std::min(old_n, max_prev_laps);
    if (n_prev >= 5 && std::abs(est / old_v - 1.0) > outlier_ratio) {
      char b[160];
      std::snprintf(b, sizeof(b), "%s 이번 값 %.4f가 누적 %.4f와 %.1f%% 달라 반영 안 함. ", name, est, old_v, (est / old_v - 1.0) * 100);
      notes += b;
      return false;
    }
    *out = (old_v * n_prev + est * N) / double(n_prev + N);
    *out_n = n_prev + N;
    return true;
  };
  r.gyro_used = merge(current.gyro_scale, current.gyro_laps, r.est.gyro_scale, &r.calib.gyro_scale, &r.calib.gyro_laps, "자이로");
  const int key = OdomCalib::speed_key(run.target_speed);
  const auto it = current.wheel_scale.find(key);
  const auto itn = current.wheel_laps.find(key);
  double wv = it != current.wheel_scale.end() ? it->second : 1.0;
  int wn = it != current.wheel_scale.end() && itn != current.wheel_laps.end() ? itn->second : 0;
  r.wheel_used = merge(wv, wn, r.est.wheel_scale, &wv, &wn, "바퀴");
  if (r.wheel_used) { r.calib.wheel_scale[key] = wv; r.calib.wheel_laps[key] = wn; }
  r.applied = r.gyro_used || r.wheel_used;
  r.after = evaluate(run, r.calib, map, max_range);
  char b[200];
  std::snprintf(b, sizeof(b), "보정 %s: 자이로 ×%.4f (누적 %d바퀴), 거리 ×%.4f @%.2f m/s. ", r.applied ? "반영" : "안 함", r.calib.gyro_scale,
                r.calib.gyro_laps, r.calib.wheel_scale_for(run.target_speed), run.target_speed);
  r.message = b + notes;
  return r;
}

// ---------------------------------------------------------------- 실시간
void SlamCore::set_config(const SlamConfig & c) { std::lock_guard<std::mutex> l(m_); cfg_ = c; run_.stopline_correction = c.stopline_correction; }
void SlamCore::set_calib(const OdomCalib & c) { std::lock_guard<std::mutex> l(m_); calib_ = c; }
OdomCalib SlamCore::calib() const { std::lock_guard<std::mutex> l(m_); return calib_; }

double SlamCore::arc_at_dist(double dist) const {
  return map_.wrap_s(anchor_s_ + (run_.cw ? -1.0 : 1.0) * (dist - anchor_dist_));
}

Pose SlamCore::snap_of(const Pose & f) const {
  Vec2 tg;
  const Vec2 c = map_.center_at(arc_at_dist(f.dist), &tg);
  const Vec2 d = run_.cw ? Vec2{-tg.x, -tg.y} : tg, left{-d.y, d.x};
  const double lat = live_.lat_valid ? live_.lat : 0.0;
  return {f.t, c.x + left.x * lat, c.y + left.y * lat, f.yaw, f.dist};
}

void SlamCore::begin_run(double target_speed, int laps, const Pose & start, bool cw) {
  std::lock_guard<std::mutex> l(m_);
  run_ = RunData{};
  run_.start = start;
  run_.start.dist = 0;
  run_.gyro_bias0 = bias_init_ ? bias_ema_ : 0.0;
  run_.target_speed = target_speed;
  run_.laps_target = laps;
  run_.stopline_correction = cfg_.stopline_correction;
  run_.cw = cw;
  run_.crossing_latency = calib_.latency;
  live_ = LiveState{};
  live_.running = true;
  live_.waiting_s = true;
  live_.cw = cw;
  lane_rel_n_ = 0; lane_rel_yaw_ = lane_rel_lat_ = 0;
  live_.laps_target = laps;
  live_.target_speed = target_speed;
  live_.free_pose = run_.start;
  live_.lap_length = map_.center_length();
  live_.bias0 = run_.gyro_bias0;
  s_stop_ = map_.center_project(map_.stop_center());
  anchor_s_ = map_.center_project({start.x, start.y});
  anchor_dist_ = 0;
  {   // 출발 옆 위치 (설정한 출발 위치 기준, 차선이 보이면 곧 BEV 값으로 바뀜)
    Vec2 tg;
    const Vec2 c = map_.center_at(anchor_s_, &tg);
    const Vec2 d = cw ? Vec2{-tg.x, -tg.y} : tg;
    live_.lat = (start.x - c.x) * -d.y + (start.y - c.y) * d.x;
    live_.lat_valid = true;
  }
  live_.s_arc = anchor_s_;
  live_.pose = snap_of(run_.start);
  hist_.clear();
  hist_.push_back(run_.start);
  track_.assign(1, live_.pose);
  free_track_.assign(1, run_.start);
  track_last_t_ = start.t;
  new_pts_.clear(); new_side_.clear();
  stop_n_ = 0; armed_dist_ = -1; last_cross_dist_ = 0;
  laps_.clear(); corr_.clear();
  lap_sq_ = lap_turn_ = lap_lat_sum_ = 0; lap_n_ = lap_lat_n_ = 0;
  finish_reported_ = false;
  last_rejected_ = false;
  last_gyro_t_ = start.t;
}

void SlamCore::set_run_params(const std::map<std::string, double> & p) { std::lock_guard<std::mutex> l(m_); run_.params = p; }

void SlamCore::end_run() { std::lock_guard<std::mutex> l(m_); live_.running = false; }

void SlamCore::on_speed(double v, double t) {
  std::lock_guard<std::mutex> l(m_);
  last_v_ = v;
  if (live_.running && !live_.waiting_s) run_.speed.push_back({t, v});
}

void SlamCore::on_gyro(double w, double t) {
  std::lock_guard<std::mutex> l(m_);
  if (!live_.running) {   // 정지 중 영점 (다음 출발의 b0)
    if (std::abs(last_v_) < 0.02 && last_gyro_t_ > 0) {
      const double dt = std::clamp(t - last_gyro_t_, 0.0, 0.05), a = dt / (2.0 + dt);
      bias_ema_ = bias_init_ ? bias_ema_ + a * (w - bias_ema_) : w;
      bias_init_ = true;
    }
    last_gyro_t_ = t;
    return;
  }
  if (!live_.waiting_s) run_.gyro.push_back({t, w});
  last_w_ = w;
  step(t);
}

void SlamCore::on_imu(const ImuSample & s) {
  std::lock_guard<std::mutex> l(m_);
  if (live_.running && !live_.waiting_s && run_.imu.size() < 400000) run_.imu.push_back(s);
}

void SlamCore::step(double t) {
  const double dt = std::clamp(t - last_gyro_t_, 0.0, 0.05);
  last_gyro_t_ = t;
  Pose & p = live_.free_pose;
  const double v = last_v_ * calib_.wheel_scale_for(last_v_);
  const double w = calib_.gyro_scale * (last_w_ - run_.gyro_bias0 - calib_.gyro_bias);
  const double ym = p.yaw + 0.5 * w * dt;
  p.x += v * std::cos(ym) * dt;
  p.y += v * std::sin(ym) * dt;
  p.yaw = wrap(p.yaw + w * dt);
  p.dist += v * dt;
  p.t = t;
  live_.lap_dist += v * dt;
  if (!live_.waiting_s) lap_turn_ += w * dt;
  hist_.push_back(p);
  while (hist_.size() > 2 && hist_.front().t < t - 3.0) hist_.pop_front();
  live_.s_arc = arc_at_dist(p.dist);
  live_.pose = snap_of(p);
  if (t - track_last_t_ >= 0.05) { track_.push_back(live_.pose); free_track_.push_back(p); track_last_t_ = t; }
  if (armed_dist_ > 0 && p.dist >= armed_dist_) on_crossing(t);
}

void SlamCore::on_crossing(double t) {
  armed_dist_ = -1;
  stop_n_ = 0;
  Pose & p = live_.free_pose;
  const Vec2 S = map_.stop_center(), dir = map_.travel_dir_at(S, run_.cw);
  const double Lc = map_.center_length();
  if (live_.waiting_s) {   // 처음 지나는 정지선 S = 기록 시작점: 여기서부터 바퀴·기록·매핑
    const Vec2 rel = take_lane_rel(p.yaw);
    anchor_at_stop(p, rel, map_, run_.cw);
    p.dist = 0;
    p.t = t;
    const double bias0 = run_.gyro_bias0, v = run_.target_speed, lat = run_.crossing_latency;
    const int laps = run_.laps_target;
    const bool corr = run_.stopline_correction, cw = run_.cw;
    auto params = run_.params;
    run_ = RunData{};
    run_.start = p;
    run_.gyro_bias0 = bias0; run_.target_speed = v; run_.crossing_latency = lat; run_.laps_target = laps; run_.stopline_correction = corr; run_.cw = cw;
    run_.params = std::move(params);
    run_.start_rel = rel;
    anchor_s_ = s_stop_; anchor_dist_ = 0;
    live_.s_arc = s_stop_;
    live_.pose = snap_of(p);
    hist_.clear(); hist_.push_back(p);
    track_.assign(1, live_.pose); free_track_.assign(1, p); track_last_t_ = t;
    last_cross_dist_ = 0; live_.lap_dist = 0;
    lap_sq_ = lap_turn_ = lap_lat_sum_ = 0; lap_n_ = lap_lat_n_ = 0;
    live_.waiting_s = false;
    return;
  }
  // 한 바퀴에 정지선은 하나: 직전 통과 후 거리가 한 바퀴 길이와 맞지 않으면 오인식으로 보고 무시
  if ((p.dist - last_cross_dist_) / std::max(1e-6, Lc) < 1.0 - cfg_.lap_window) { ++live_.rejected_lap; return; }
  LapMetrics m;
  m.lap = live_.lap + 1;
  m.dist_m = p.dist - last_cross_dist_;
  m.s_err_m = m.dist_m - Lc;
  m.length_err = m.dist_m / std::max(1e-6, Lc) - 1.0;
  m.turn_deg = (run_.cw ? -lap_turn_ : lap_turn_) * 180.0 / kPi;
  m.lat_mean_m = lap_lat_n_ ? lap_lat_sum_ / lap_lat_n_ : 0.0;
  m.along_err_m = dot({p.x - S.x, p.y - S.y}, dir);
  m.heading_err_deg = wrap(p.yaw - std::atan2(dir.y, dir.x)) * 180.0 / kPi;
  m.points = lap_n_;
  m.rms_m = lap_n_ ? std::sqrt(lap_sq_ / lap_n_) : kNaN;
  laps_.push_back(m);
  run_.crossing_t.push_back(t);
  const Vec2 rel = take_lane_rel(p.yaw);
  run_.anchor_rel.push_back(rel);
  if (cfg_.stopline_correction) {   // 자유 적분도 S에 맞춤 (앞뒤 = S, 옆·방향 = BEV 차선)
    anchor_at_stop(p, rel, map_, run_.cw);
    corr_.push_back({m.lap, -m.along_err_m});
  }
  anchor_s_ = s_stop_; anchor_dist_ = p.dist;   // 위치 추정은 항상 S에서 다시 시작
  live_.s_arc = s_stop_;
  live_.pose = snap_of(p);
  last_cross_dist_ = p.dist;
  lap_sq_ = lap_turn_ = lap_lat_sum_ = 0; lap_n_ = lap_lat_n_ = 0;
  live_.lap_dist = 0;
  live_.lap = m.lap;
  if (live_.laps_target > 0 && live_.lap >= live_.laps_target) live_.finished = true;
}

Vec2 SlamCore::take_lane_rel(double yaw_now) {
  // 잰 순간 i의 차 방향 ψi = 차선 방향 − rel_i → 통과 순간 rel = (rel_i + ψi) − ψ_now. 코너를 막 빠져나오며 차가 돌고 있어도 맞음
  Vec2 r{kNaN, 0};
  if (lane_rel_n_ > 0) r = {wrap(lane_rel_ref_ + lane_rel_yaw_ / lane_rel_n_ - yaw_now), lane_rel_lat_ / lane_rel_n_};
  lane_rel_n_ = 0; lane_rel_yaw_ = lane_rel_lat_ = 0;
  return r;
}

Pose SlamCore::pose_at(double t) const {
  if (hist_.empty()) return live_.free_pose;
  if (t <= hist_.front().t) return hist_.front();
  if (t >= hist_.back().t) return hist_.back();
  const auto it = std::lower_bound(hist_.begin(), hist_.end(), t, [](const Pose & p, double v) { return p.t < v; });
  const Pose & b = *it, & a = *std::prev(it);
  const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
  return {t, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.yaw + u * wrap(b.yaw - a.yaw), a.dist + u * (b.dist - a.dist)};
}

bool SlamCore::on_lane(const LaneObservation & lane) {
  std::lock_guard<std::mutex> l(m_);
  if (!live_.running || !lane.valid) return false;
  LaneSample s;
  s.t = lane.capture_time;
  s.stop_m = lane.stop_line_m;
  for (const auto & q : lane.left) if (q.x >= 0 && q.x <= cfg_.max_range_m) s.left.push_back(q);
  for (const auto & q : lane.right) if (q.x >= 0 && q.x <= cfg_.max_range_m) s.right.push_back(q);
  const Pose seen = pose_at(s.t + calib_.latency);   // 그 장면을 본 순간의 자유 적분 자세
  // 그 장면에서 본 정지선이 중앙선 위에서 S 근처인가 (오도메트리 누적 거리 기준이라 위치가 틀어져도 안전)
  const bool stop_valid = std::isfinite(s.stop_m) && s.stop_m > 0.15 && s.stop_m < cfg_.max_range_m;
  const bool stop_near = stop_valid && std::abs(map_.arc_diff(arc_at_dist(seen.dist + s.stop_m), s_stop_)) <= cfg_.stop_gate_m;
  {
    const auto f = fit_lane_frame(s.left, s.right, 0.15, 0.8, 0.3);
    if (f.both) live_.lane_width = std::isfinite(live_.lane_width) ? live_.lane_width + 0.05 * (f.width - live_.lane_width) : f.width;
    // 옆 위치: 차 바로 앞(0.1~0.5 m)의 차선 가운데. 한쪽만 보이면 그 선 ± 반폭.
    // 코너 진입·탈출처럼 가까운 선도 휘어 보이면(기울기 큼) 이전 값을 유지 (휜 선을 직선으로 맞추면 옆 위치가 튄다)
    const double half = 0.5 * (std::isfinite(live_.lane_width) ? live_.lane_width : map_.lane_width());
    const auto near = fit_lane_frame(s.left, s.right, 0.1, 0.5, 0.3);
    constexpr double kMaxSlope = 0.2;
    double c = kNaN;
    if (near.both && std::abs(near.angle) < kMaxSlope) c = near.center_y;
    else if (!near.both && near.left.ok && std::abs(near.left.a) < kMaxSlope && near.left.y_at(0.3) > 0.1) c = near.left.y_at(0.3) - half;
    else if (!near.both && near.right.ok && std::abs(near.right.a) < kMaxSlope && near.right.y_at(0.3) < -0.1) c = near.right.y_at(0.3) + half;
    if (std::isfinite(c) && std::abs(c) < 0.5) {
      live_.lat = live_.lat_valid ? live_.lat + 0.3 * (-c - live_.lat) : -c;
      live_.lat_valid = true;
      if (!live_.waiting_s) { lap_lat_sum_ += live_.lat; ++lap_lat_n_; }
    }
    // 정지선 S가 실제로 0.2~1.0 m 앞에 보일 때 양쪽 차선으로 차선 대비 방향·옆 위치 (통과할 때 자유 적분 자세를 다시 잡음)
    if (f.both && stop_near && s.stop_m > 0.2 && s.stop_m < 1.0) {
      const double a = f.angle + seen.yaw;   // 차선 방향 (자유 적분 좌표)
      if (lane_rel_n_ == 0) { lane_rel_ref_ = a; lane_rel_yaw_ = 0; }
      lane_rel_yaw_ += wrap(a - lane_rel_ref_);
      lane_rel_lat_ += f.center_y;
      ++lane_rel_n_;
    }
  }
  // 매핑 (화면·참고 RMS): 위치 추정 지점 + 자이로 방향. S를 지나기 전(가속 구간)에는 하지 않음
  if (!live_.waiting_s) {
    const Pose p = snap_of(seen);
    for (int side = 0; side < 2; ++side) {
      const auto & pts = side == 0 ? s.left : s.right;
      const std::size_t stride = std::max<std::size_t>(1, pts.size() / 6);
      for (std::size_t k = 0; k < pts.size(); k += stride) {
        const Vec2 w = to_world(p, pts[k]);
        const double d = std::min(kCap, map_.dist_side(side, run_.cw, w));
        lap_sq_ += d * d; ++lap_n_;
        if (new_pts_.size() < 1200) { new_pts_.push_back(w); new_side_.push_back(side); }
      }
    }
  }
  // 정지선 통과 예측: 앞차축이 정지선에 닿는 누적 거리
  if (stop_valid) {
    live_.stop_seen_m = s.stop_m;
    live_.stop_near_s = stop_near;
    if (!stop_near) {   // S가 아닌 곳: 오인식 (코너의 휜 선 등)
      if (!last_rejected_) ++live_.rejected_far;
      last_rejected_ = true;
    } else {
      last_rejected_ = false;
      const double d_cross = seen.dist + s.stop_m;
      if (stop_n_ > 0 && std::abs(d_cross - stop_cand_) < cfg_.stop_tolerance_m) { stop_cand_ = d_cross; ++stop_n_; }
      else { stop_cand_ = d_cross; stop_n_ = 1; }
      if (stop_n_ >= cfg_.stop_confirm) armed_dist_ = stop_cand_;
    }
  } else {
    live_.stop_seen_m = kNaN;
    last_rejected_ = false;
  }
  if (!live_.waiting_s) run_.lanes.push_back(std::move(s));
  if (live_.finished && !finish_reported_) { finish_reported_ = true; return true; }
  return false;
}

LiveState SlamCore::live() const { std::lock_guard<std::mutex> l(m_); return live_; }
std::vector<LapMetrics> SlamCore::lap_metrics() const { std::lock_guard<std::mutex> l(m_); return laps_; }
std::vector<std::pair<int, double>> SlamCore::corrections() const { std::lock_guard<std::mutex> l(m_); return corr_; }
RunData SlamCore::run_data() const { std::lock_guard<std::mutex> l(m_); return run_; }

std::vector<Vec2> SlamCore::take_map_points(std::vector<int> * side) {
  std::lock_guard<std::mutex> l(m_);
  std::vector<Vec2> out;
  out.swap(new_pts_);
  if (side) side->swap(new_side_);
  new_side_.clear();
  return out;
}

std::vector<Pose> SlamCore::track_since(std::size_t * cursor) const {
  std::lock_guard<std::mutex> l(m_);
  if (*cursor > track_.size()) *cursor = 0;
  std::vector<Pose> out(track_.begin() + long(*cursor), track_.end());
  *cursor = track_.size();
  return out;
}

std::vector<Pose> SlamCore::free_since(std::size_t * cursor) const {
  std::lock_guard<std::mutex> l(m_);
  if (*cursor > free_track_.size()) *cursor = 0;
  std::vector<Pose> out(free_track_.begin() + long(*cursor), free_track_.end());
  *cursor = free_track_.size();
  return out;
}

}  // namespace semi_slam
