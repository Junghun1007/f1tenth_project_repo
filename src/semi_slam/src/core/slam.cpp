#include "semi_slam/core/slam.hpp"
#include "semi_slam/core/json_writer.hpp"
#include "semi_slam/core/mini_json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>

namespace semi_slam {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kLaneHalf = 0.325;
constexpr double kCap = 0.15;          // 차선 점 오차 상한 (오인식 점이 보정을 끌고 가지 않게)
double wrap(double a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }
double dot(const Vec2 & a, const Vec2 & b) { return a.x * b.x + a.y * b.y; }
Vec2 to_world(const Pose & p, const Vec2 & q) {
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y};
}
}  // namespace

// ---------------------------------------------------------------- 보정 값
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

void OdomCalib::set_wheel_scale(double v, double s) { wheel_scale[int(std::lround(std::abs(v) * 20.0)) * 5] = s; }

std::string OdomCalib::to_json() const {
  JsonWriter w;
  w.begin_object().key("gyro_scale").value(gyro_scale, 6).key("gyro_bias").value(gyro_bias, 6).key("latency").value(latency, 4);
  w.key("wheel_scale").begin_object();
  for (const auto & [k, s] : wheel_scale) w.key(std::to_string(k).c_str()).value(s, 6);
  w.end_object().end_object();
  return w.str();
}

OdomCalib OdomCalib::from_json(const std::string & text) {
  OdomCalib c;
  const auto j = parse_json(text);
  if (!j || j->kind != JsonValue::Object) return c;
  auto num = [&](const char * k, double & out) { const auto * v = j->get(k); if (v && v->kind == JsonValue::Number) out = v->num; };
  num("gyro_scale", c.gyro_scale);
  num("gyro_bias", c.gyro_bias);
  num("latency", c.latency);
  if (const auto * ws = j->get("wheel_scale"); ws && ws->kind == JsonValue::Object)
    for (const auto & [k, v] : ws->obj) if (v->kind == JsonValue::Number) c.wheel_scale[std::stoi(k)] = v->num;
  return c;
}

// ---------------------------------------------------------------- 적분
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
  tr.poses.push_back(p);
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S);
  std::size_t si = 0, ci = 0;
  double v_raw = 0, last_t = p.t;
  const double shift = calib.latency - run.crossing_latency;   // 통과 시각 = 정지선을 본 영상 시각 기준이라 시간차가 바뀌면 같이 옮김
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
    while (ci < run.crossing_t.size() && run.crossing_t[ci] + shift <= t) {   // 정지선 S 통과: 진행 방향 위치를 S에 맞춤
      if (correction) {
        const double along = dot({p.x - S.x, p.y - S.y}, dir);
        p.x -= along * dir.x;
        p.y -= along * dir.y;
      }
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

// ---------------------------------------------------------------- 평가
Evaluation evaluate(const RunData & run, const OdomCalib & calib, const TrackMap & map, double max_range) {
  Evaluation e;
  const auto tr = integrate(run, calib, map, run.stopline_correction);
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S);
  const std::size_t nl = run.crossing_t.size() + 1;
  std::vector<double> cross = run.crossing_t;
  for (auto & c : cross) c += calib.latency - run.crossing_latency;
  std::vector<double> lap_sq(nl, 0.0);
  std::vector<int> lap_n(nl, 0);
  double sq = 0, ssq = 0;
  int ns = 0;
  for (std::size_t k = 0; k < run.lanes.size(); k += 2) {   // 30 Hz면 충분
    const auto & L = run.lanes[k];
    const Pose p = tr.at(L.t + calib.latency);
    const std::size_t lap = std::size_t(std::upper_bound(cross.begin(), cross.end(), L.t + calib.latency) - cross.begin());
    for (int side = 0; side < 2; ++side)
      for (const auto & q : side == 0 ? L.left : L.right) {
        if (q.x < 0 || q.x > max_range) continue;
        const Vec2 w = to_world(p, q);
        const double d = std::min(kCap, side == 0 ? map.dist_inner(w) : map.dist_outer(w));
        sq += d * d; lap_sq[lap] += d * d; ++lap_n[lap]; ++e.points;
      }
    if (std::isfinite(L.stop_m) && L.stop_m > 0.15 && L.stop_m < max_range) {
      const Vec2 w = to_world(p, {L.stop_m, 0.0});
      if (std::hypot(w.x - S.x, w.y - S.y) < 0.7) { const double a = dot({w.x - S.x, w.y - S.y}, dir); ssq += a * a; ++ns; }
    }
  }
  e.rms_m = e.points ? std::sqrt(sq / e.points) : std::numeric_limits<double>::quiet_NaN();
  e.stop_rms_m = ns ? std::sqrt(ssq / ns) : 0.0;
  e.accuracy = e.points ? std::max(0.0, 1.0 - e.rms_m / kLaneHalf) : 0.0;
  e.cost = (e.points ? sq / e.points : 1.0) + (ns ? ssq / ns : 0.0);
  const Vec2 st{run.start.x, run.start.y};
  double prev_d = 0;
  for (std::size_t k = 0; k < run.crossing_t.size(); ++k) {
    const Pose p = tr.at(cross[k] - 1e-3);   // 통과 직전 (보정 전) 위치
    LapMetrics m;
    m.lap = int(k) + 1;
    m.along_err_m = dot({p.x - S.x, p.y - S.y}, dir);
    m.heading_err_deg = wrap(p.yaw - std::atan2(dir.y, dir.x)) * 180.0 / kPi;
    const double expected = k == 0 ? map.lap_length() - dot({st.x - S.x, st.y - S.y}, dir) : map.lap_length();
    m.length_err = (p.dist - prev_d) / std::max(1e-6, expected) - 1.0;
    prev_d = p.dist;
    m.points = lap_n[k];
    m.rms_m = lap_n[k] ? std::sqrt(lap_sq[k] / lap_n[k]) : std::numeric_limits<double>::quiet_NaN();
    e.laps.push_back(m);
  }
  return e;
}

// ---------------------------------------------------------------- 보정 (Nelder–Mead)
CalibResult calibrate(const RunData & run, const OdomCalib & start, const TrackMap & map, double max_range) {
  CalibResult r;
  r.before = evaluate(run, start, map, max_range);
  if (run.lanes.size() < 50 || r.before.points < 200) {
    r.calib = start;
    r.after = r.before;
    r.message = "기록이 너무 적어 보정하지 않음";
    return r;
  }
  using X = std::array<double, 4>;   // 거리 배율, 자이로 배율, 잔여 영점, 시간차
  const X lo{0.80, 0.90, -0.05, -0.05}, hi{1.20, 1.10, 0.05, 0.20};
  auto clampx = [&](X x) { for (int i = 0; i < 4; ++i) x[i] = std::clamp(x[i], lo[i], hi[i]); return x; };
  auto make = [&](const X & x) {
    OdomCalib c = start;
    c.wheel_scale.clear();
    c.wheel_scale[100] = x[0];   // 적합 중에는 모든 속도에 같은 배율
    c.gyro_scale = x[1];
    c.gyro_bias = x[2];
    c.latency = x[3];
    return c;
  };
  auto f = [&](const X & x) { ++r.evaluations; return evaluate(run, make(clampx(x)), map, max_range).cost; };
  X best_x = clampx(X{start.wheel_scale_for(run.target_speed), start.gyro_scale, start.gyro_bias, start.latency});
  double best_f = f(best_x);
  // Nelder–Mead를 찾은 점에서 새 단체로 몇 번 다시 시작 (지연처럼 비용 면이 평평한 값이 중간에 멈추지 않게)
  for (int restart = 0; restart < 3; ++restart) {
    const double k = restart == 0 ? 1.0 : 0.5;
    const X step{0.02 * k, 0.01 * k, 0.004 * k, 0.03 * k};
    std::array<X, 5> s;
    std::array<double, 5> fv;
    s[0] = best_x;
    fv[0] = best_f;
    for (int i = 0; i < 4; ++i) { s[i + 1] = s[0]; s[i + 1][i] += step[i]; s[i + 1] = clampx(s[i + 1]); fv[i + 1] = f(s[i + 1]); }
    for (int it = 0; it < 160; ++it) {
      std::array<int, 5> o{0, 1, 2, 3, 4};
      std::sort(o.begin(), o.end(), [&](int a, int b) { return fv[a] < fv[b]; });
      std::array<X, 5> s2; std::array<double, 5> f2;
      for (int i = 0; i < 5; ++i) { s2[i] = s[o[i]]; f2[i] = fv[o[i]]; }
      s = s2; fv = f2;
      if (std::abs(fv[4] - fv[0]) < 1e-10) break;
      X c{};
      for (int i = 0; i < 4; ++i) for (int d = 0; d < 4; ++d) c[d] += s[i][d] / 4.0;
      auto lerp = [&](double a) { X x; for (int d = 0; d < 4; ++d) x[d] = c[d] + a * (s[4][d] - c[d]); return clampx(x); };
      const X xr = lerp(-1.0); const double fr = f(xr);
      if (fr < fv[0]) { const X xe = lerp(-2.0); const double fe = f(xe); if (fe < fr) { s[4] = xe; fv[4] = fe; } else { s[4] = xr; fv[4] = fr; } continue; }
      if (fr < fv[3]) { s[4] = xr; fv[4] = fr; continue; }
      const X xc = lerp(0.5); const double fc = f(xc);
      if (fc < fv[4]) { s[4] = xc; fv[4] = fc; continue; }
      for (int i = 1; i < 5; ++i) { for (int d = 0; d < 4; ++d) s[i][d] = s[0][d] + 0.5 * (s[i][d] - s[0][d]); s[i] = clampx(s[i]); fv[i] = f(s[i]); }
    }
    const int bi = int(std::min_element(fv.begin(), fv.end()) - fv.begin());
    if (fv[bi] < best_f) { best_f = fv[bi]; best_x = s[bi]; }
  }
  const X xb = best_x;
  r.calib = start;
  r.calib.set_wheel_scale(run.target_speed, xb[0]);   // 이 주행 속도의 거리 배율만 갱신
  r.calib.gyro_scale = xb[1];
  r.calib.gyro_bias = xb[2];
  r.calib.latency = xb[3];
  r.after = evaluate(run, r.calib, map, max_range);
  r.improved = r.after.cost < r.before.cost * 0.99 && r.after.rms_m <= r.before.rms_m;
  r.message = r.improved ? "보정 적용" : "오차가 줄지 않아 적용하지 않음";
  return r;
}

// ---------------------------------------------------------------- 실시간
void SlamCore::set_config(const SlamConfig & c) { std::lock_guard<std::mutex> l(m_); cfg_ = c; run_.stopline_correction = c.stopline_correction; }
void SlamCore::set_calib(const OdomCalib & c) { std::lock_guard<std::mutex> l(m_); calib_ = c; }
OdomCalib SlamCore::calib() const { std::lock_guard<std::mutex> l(m_); return calib_; }

void SlamCore::begin_run(double target_speed, int laps, const Pose & start) {
  std::lock_guard<std::mutex> l(m_);
  run_ = RunData{};
  run_.start = start;
  run_.start.dist = 0;
  run_.gyro_bias0 = bias_init_ ? bias_ema_ : 0.0;
  run_.target_speed = target_speed;
  run_.laps_target = laps;
  run_.stopline_correction = cfg_.stopline_correction;
  run_.crossing_latency = calib_.latency;
  live_ = LiveState{};
  live_.running = true;
  live_.laps_target = laps;
  live_.target_speed = target_speed;
  live_.pose = run_.start;
  live_.lap_length = map_.lap_length();
  live_.bias0 = run_.gyro_bias0;
  hist_.clear();
  hist_.push_back(run_.start);
  track_.clear();
  track_.push_back(run_.start);
  track_last_t_ = start.t;
  new_pts_.clear(); new_side_.clear();
  stop_n_ = 0; armed_dist_ = -1; last_cross_dist_ = 0;
  laps_.clear(); corr_.clear();
  lap_sq_ = 0; lap_n_ = 0;
  finish_reported_ = false;
  last_gyro_t_ = start.t;
}

void SlamCore::end_run() { std::lock_guard<std::mutex> l(m_); live_.running = false; }

void SlamCore::on_speed(double v, double t) {
  std::lock_guard<std::mutex> l(m_);
  last_v_ = v;
  if (live_.running) run_.speed.push_back({t, v});
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
  run_.gyro.push_back({t, w});
  last_w_ = w;
  step(t);
}

void SlamCore::step(double t) {
  const double dt = std::clamp(t - last_gyro_t_, 0.0, 0.05);
  last_gyro_t_ = t;
  Pose & p = live_.pose;
  const double v = last_v_ * calib_.wheel_scale_for(last_v_);
  const double w = calib_.gyro_scale * (last_w_ - run_.gyro_bias0 - calib_.gyro_bias);
  const double ym = p.yaw + 0.5 * w * dt;
  p.x += v * std::cos(ym) * dt;
  p.y += v * std::sin(ym) * dt;
  p.yaw = wrap(p.yaw + w * dt);
  p.dist += v * dt;
  p.t = t;
  live_.lap_dist += v * dt;
  hist_.push_back(p);
  while (hist_.size() > 2 && hist_.front().t < t - 3.0) hist_.pop_front();
  if (t - track_last_t_ >= 0.05) { track_.push_back(p); track_last_t_ = t; }
  if (armed_dist_ > 0 && p.dist >= armed_dist_) on_crossing(t);
}

void SlamCore::on_crossing(double t) {
  armed_dist_ = -1;
  stop_n_ = 0;
  Pose & p = live_.pose;
  if (p.dist - last_cross_dist_ < cfg_.min_lap_fraction * map_.lap_length()) return;   // 출발선 T1 등 바로 뒤의 선
  const Vec2 S = map_.stop_center(), dir = map_.travel_dir_at(S);
  LapMetrics m;
  m.lap = live_.lap + 1;
  m.along_err_m = dot({p.x - S.x, p.y - S.y}, dir);
  m.heading_err_deg = wrap(p.yaw - std::atan2(dir.y, dir.x)) * 180.0 / kPi;
  const double expected = live_.lap == 0 ? map_.lap_length() - dot({run_.start.x - S.x, run_.start.y - S.y}, dir) : map_.lap_length();
  m.length_err = (p.dist - last_cross_dist_) / std::max(1e-6, expected) - 1.0;
  m.points = lap_n_;
  m.rms_m = lap_n_ ? std::sqrt(lap_sq_ / lap_n_) : std::numeric_limits<double>::quiet_NaN();
  laps_.push_back(m);
  run_.crossing_t.push_back(t);
  if (cfg_.stopline_correction) {
    p.x -= m.along_err_m * dir.x;
    p.y -= m.along_err_m * dir.y;
    corr_.push_back({m.lap, -m.along_err_m});
  }
  last_cross_dist_ = p.dist;
  lap_sq_ = 0; lap_n_ = 0;
  live_.lap_dist = 0;
  live_.lap = m.lap;
  if (live_.laps_target > 0 && live_.lap >= live_.laps_target) live_.finished = true;
}

Pose SlamCore::pose_at(double t) const {
  if (hist_.empty()) return live_.pose;
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
  // 매핑 (그 장면을 본 위치 = 촬영 시각 + 시간차)
  const Pose p = pose_at(s.t + calib_.latency);
  for (int side = 0; side < 2; ++side) {
    const auto & pts = side == 0 ? s.left : s.right;
    const std::size_t stride = std::max<std::size_t>(1, pts.size() / 6);
    for (std::size_t k = 0; k < pts.size(); k += stride) {
      const Vec2 w = to_world(p, pts[k]);
      const double d = std::min(kCap, side == 0 ? map_.dist_inner(w) : map_.dist_outer(w));
      lap_sq_ += d * d; ++lap_n_;
      if (new_pts_.size() < 1200) { new_pts_.push_back(w); new_side_.push_back(side); }
    }
  }
  // 정지선 통과 예측: 앞차축이 정지선에 닿는 누적 거리
  if (std::isfinite(s.stop_m) && s.stop_m > 0.15 && s.stop_m < cfg_.max_range_m) {
    const double d_cross = pose_at(s.t + calib_.latency).dist + s.stop_m;   // 그 장면을 본 위치 기준
    if (stop_n_ > 0 && std::abs(d_cross - stop_cand_) < cfg_.stop_tolerance_m) { stop_cand_ = 0.5 * (stop_cand_ + d_cross); ++stop_n_; }
    else { stop_cand_ = d_cross; stop_n_ = 1; }
    if (stop_n_ >= cfg_.stop_confirm) armed_dist_ = stop_cand_;
  }
  run_.lanes.push_back(std::move(s));
  if (live_.finished && !finish_reported_) { finish_reported_ = true; return true; }   // 통과는 IMU 적분 중에 일어나므로 여기서 한 번 알림
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

}  // namespace semi_slam
