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

Vec2 OdomCalib::bev_point(const Vec2 & q) const {
  const double c = std::cos(cam_yaw), s = std::sin(cam_yaw), y = q.y * bev_scale;
  return {c * q.x - s * y, s * q.x + c * y};
}

// 정지선 S에서 BEV로 잰 차선 대비 값(원값)으로 차량 자세를 다시 잡는다: 앞뒤 = S, 옆 = 두 선 가운데 기준, 방향 = 차선 방향 기준
static void anchor_at_stop(Pose & p, const Vec2 & rel, const OdomCalib & c, const TrackMap & map, bool cw) {
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S, cw), left{-dir.y, dir.x};
  const double along = (p.x - S.x) * dir.x + (p.y - S.y) * dir.y;
  p.x -= along * dir.x;
  p.y -= along * dir.y;
  if (!std::isfinite(rel.x)) return;
  const double lat = -rel.y * c.bev_scale;   // 두 선 가운데가 차 기준 왼쪽이면 차는 가운데보다 오른쪽
  p.x = S.x + left.x * lat;
  p.y = S.y + left.y * lat;
  p.yaw = std::atan2(dir.y, dir.x) - (rel.x + c.cam_yaw);   // 차선이 차 기준 반시계로 보이면 차는 시계 방향으로 틀어진 것
}

void OdomCalib::set_wheel_scale(double v, double s) { wheel_scale[int(std::lround(std::abs(v) * 20.0)) * 5] = s; }

std::string OdomCalib::to_json() const {
  JsonWriter w;
  w.begin_object().key("gyro_scale").value(gyro_scale, 6).key("gyro_bias").value(gyro_bias, 6).key("latency").value(latency, 4)
   .key("cam_yaw").value(cam_yaw, 6).key("bev_scale").value(bev_scale, 5);
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
  num("cam_yaw", c.cam_yaw);
  num("bev_scale", c.bev_scale);
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
  if (std::isfinite(run.start_rel.x)) anchor_at_stop(p, run.start_rel, calib, map, run.cw);   // 기록 시작점도 후보 카메라 값으로 다시
  tr.poses.push_back(p);
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
      if (correction) anchor_at_stop(p, ci < run.anchor_rel.size() ? run.anchor_rel[ci] : Vec2{std::nan(""), 0}, calib, map, run.cw);
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
Evaluation evaluate(const RunData & run, const OdomCalib & calib, const TrackMap & map, double max_range, double cap) {
  Evaluation e;
  const auto tr = integrate(run, calib, map, run.stopline_correction);
  const Vec2 S = map.stop_center(), dir = map.travel_dir_at(S, run.cw);
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
        const Vec2 w = to_world(p, calib.bev_point(q));
        const double d = std::min(cap, map.dist_side(side, run.cw, w));
        sq += d * d; lap_sq[lap] += d * d; ++lap_n[lap]; ++e.points;
      }
    if (std::isfinite(L.stop_m) && L.stop_m > 0.15 && L.stop_m < max_range) {
      const Vec2 w = to_world(p, {L.stop_m, 0.0});
      if (std::hypot(w.x - S.x, w.y - S.y) < 0.7) { const double a = dot({w.x - S.x, w.y - S.y}, dir); ssq += a * a; ++ns; }
    }
  }
  e.rms_m = e.points ? std::sqrt(sq / e.points) : std::numeric_limits<double>::quiet_NaN();
  e.stop_rms_m = ns ? std::sqrt(ssq / ns) : 0.0;
  e.accuracy = e.points ? std::max(0.0, 1.0 - e.rms_m / (0.5 * map.lane_width())) : 0.0;
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
  // x = 거리 배율, 자이로 배율, 잔여 영점, 시간차, 카메라 틀어짐, BEV 가로 축척
  constexpr int N = 6;
  using X = std::array<double, N>;
  const X lo{0.80, 0.90, -0.05, -0.05, -0.10, 0.80}, hi{1.20, 1.10, 0.05, 0.20, 0.10, 1.25};
  auto clampx = [&](X x) { for (int i = 0; i < N; ++i) x[i] = std::clamp(x[i], lo[i], hi[i]); return x; };
  auto make = [&](const X & x) {
    OdomCalib c = start;
    c.wheel_scale.clear();
    c.wheel_scale[100] = x[0];   // 적합 중에는 모든 속도에 같은 배율
    c.gyro_scale = x[1]; c.gyro_bias = x[2]; c.latency = x[3]; c.cam_yaw = x[4]; c.bev_scale = x[5];
    return c;
  };
  double cap = 0.6;
  auto f = [&](const X & x) { ++r.evaluations; return evaluate(run, make(clampx(x)), map, max_range, cap).cost; };
  X best_x = clampx(X{start.wheel_scale_for(run.target_speed), start.gyro_scale, start.gyro_bias, start.latency, start.cam_yaw, start.bev_scale});
  // 오차 상한을 크게 시작해 줄여 간다: 처음부터 15 cm로 자르면 큰 오차에서 비용이 평평해 방향을 못 잡는다
  for (double stage_cap : {0.6, 0.3, 0.15}) {
    cap = stage_cap;
    double best_f = f(best_x);
    for (int restart = 0; restart < 2; ++restart) {   // 찾은 점에서 새 단체로 다시 (평평한 방향에서 멈추지 않게)
      const double k = restart == 0 ? 1.0 : 0.4;
      const X step{0.03 * k, 0.015 * k, 0.005 * k, 0.03 * k, 0.03 * k, 0.05 * k};
      std::array<X, N + 1> s;
      std::array<double, N + 1> fv;
      s[0] = best_x; fv[0] = best_f;
      for (int i = 0; i < N; ++i) { s[i + 1] = s[0]; s[i + 1][i] += step[i]; s[i + 1] = clampx(s[i + 1]); fv[i + 1] = f(s[i + 1]); }
      for (int it = 0; it < 200; ++it) {
        std::array<int, N + 1> o;
        for (int i = 0; i <= N; ++i) o[i] = i;
        std::sort(o.begin(), o.end(), [&](int a, int b) { return fv[a] < fv[b]; });
        std::array<X, N + 1> s2; std::array<double, N + 1> f2;
        for (int i = 0; i <= N; ++i) { s2[i] = s[o[i]]; f2[i] = fv[o[i]]; }
        s = s2; fv = f2;
        if (std::abs(fv[N] - fv[0]) < 1e-10) break;
        X c{};
        for (int i = 0; i < N; ++i) for (int d = 0; d < N; ++d) c[d] += s[i][d] / N;
        auto lerp = [&](double a) { X x; for (int d = 0; d < N; ++d) x[d] = c[d] + a * (s[N][d] - c[d]); return clampx(x); };
        const X xr = lerp(-1.0); const double fr = f(xr);
        if (fr < fv[0]) { const X xe = lerp(-2.0); const double fe = f(xe); if (fe < fr) { s[N] = xe; fv[N] = fe; } else { s[N] = xr; fv[N] = fr; } continue; }
        if (fr < fv[N - 1]) { s[N] = xr; fv[N] = fr; continue; }
        const X xc = lerp(0.5); const double fc = f(xc);
        if (fc < fv[N]) { s[N] = xc; fv[N] = fc; continue; }
        for (int i = 1; i <= N; ++i) { for (int d = 0; d < N; ++d) s[i][d] = s[0][d] + 0.5 * (s[i][d] - s[0][d]); s[i] = clampx(s[i]); fv[i] = f(s[i]); }
      }
      const int bi = int(std::min_element(fv.begin(), fv.end()) - fv.begin());
      if (fv[bi] < best_f) { best_f = fv[bi]; best_x = s[bi]; }
    }
  }
  const X xb = best_x;
  r.calib = start;
  r.calib.set_wheel_scale(run.target_speed, xb[0]);   // 이 주행 속도의 거리 배율만 갱신
  r.calib.gyro_scale = xb[1]; r.calib.gyro_bias = xb[2]; r.calib.latency = xb[3]; r.calib.cam_yaw = xb[4]; r.calib.bev_scale = xb[5];
  r.after = evaluate(run, r.calib, map, max_range);
  r.improved = r.after.cost < r.before.cost * 0.99 && r.after.rms_m <= r.before.rms_m;
  r.message = r.improved ? "보정 적용" : "오차가 줄지 않아 적용하지 않음";
  return r;
}

// ---------------------------------------------------------------- 실시간
void SlamCore::set_config(const SlamConfig & c) { std::lock_guard<std::mutex> l(m_); cfg_ = c; run_.stopline_correction = c.stopline_correction; }
void SlamCore::set_calib(const OdomCalib & c) { std::lock_guard<std::mutex> l(m_); calib_ = c; }
OdomCalib SlamCore::calib() const { std::lock_guard<std::mutex> l(m_); return calib_; }

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
  live_.cw = cw;
  live_.running = true;
  live_.waiting_s = true;
  lane_rel_n_ = 0; lane_rel_yaw_ = lane_rel_lat_ = 0;
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
  last_rejected_ = false;
  last_gyro_t_ = start.t;
}

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
  const Vec2 S = map_.stop_center(), dir = map_.travel_dir_at(S, run_.cw);
  if (live_.waiting_s) {   // 처음 지나는 정지선 S = 기록 시작점: 진행 방향 위치를 S에 맞추고 여기서부터 바퀴·기록·매핑
    const Vec2 rel = take_lane_rel();   // 가속 구간 적분 오차 대신 BEV 차선으로 잰 방향·옆 위치 (S 근처 직선)
    anchor_at_stop(p, rel, calib_, map_, run_.cw);
    p.dist = 0;
    p.t = t;
    const double bias0 = run_.gyro_bias0, v = run_.target_speed, lat = run_.crossing_latency;
    const int laps = run_.laps_target;
    const bool corr = run_.stopline_correction, cw = run_.cw;
    run_ = RunData{};
    run_.start = p;
    run_.gyro_bias0 = bias0; run_.target_speed = v; run_.crossing_latency = lat; run_.laps_target = laps; run_.stopline_correction = corr; run_.cw = cw;
    run_.start_rel = rel;
    hist_.clear(); hist_.push_back(p);
    track_.clear(); track_.push_back(p); track_last_t_ = t;
    last_cross_dist_ = 0; live_.lap_dist = 0; lap_sq_ = 0; lap_n_ = 0;
    live_.waiting_s = false;
    return;
  }
  // 한 바퀴에 정지선은 하나: 직전 통과 후 거리가 한 바퀴 길이와 맞지 않으면 오인식으로 보고 무시
  const double expected_len = live_.lap == 0 ? map_.lap_length() - dot({run_.start.x - S.x, run_.start.y - S.y}, dir) : map_.lap_length();
  if ((p.dist - last_cross_dist_) / std::max(1e-6, expected_len) < 1.0 - cfg_.lap_window) { ++live_.rejected_lap; return; }
  LapMetrics m;
  m.lap = live_.lap + 1;
  m.along_err_m = dot({p.x - S.x, p.y - S.y}, dir);
  m.heading_err_deg = wrap(p.yaw - std::atan2(dir.y, dir.x)) * 180.0 / kPi;
  m.length_err = (p.dist - last_cross_dist_) / std::max(1e-6, expected_len) - 1.0;
  m.points = lap_n_;
  m.rms_m = lap_n_ ? std::sqrt(lap_sq_ / lap_n_) : std::numeric_limits<double>::quiet_NaN();
  laps_.push_back(m);
  run_.crossing_t.push_back(t);
  const Vec2 rel = take_lane_rel();
  run_.anchor_rel.push_back(rel);
  if (cfg_.stopline_correction) {   // 앞뒤 = S, 옆·방향 = BEV 차선 (S 근처 직선에서 잰 값)
    anchor_at_stop(p, rel, calib_, map_, run_.cw);
    corr_.push_back({m.lap, -m.along_err_m});
  }
  last_cross_dist_ = p.dist;
  lap_sq_ = 0; lap_n_ = 0;
  live_.lap_dist = 0;
  live_.lap = m.lap;
  if (live_.laps_target > 0 && live_.lap >= live_.laps_target) live_.finished = true;
}

Vec2 SlamCore::take_lane_rel() {
  Vec2 r{std::numeric_limits<double>::quiet_NaN(), 0};
  if (lane_rel_n_ > 0) r = {lane_rel_yaw_ / lane_rel_n_, lane_rel_lat_ / lane_rel_n_};
  lane_rel_n_ = 0; lane_rel_yaw_ = lane_rel_lat_ = 0;
  return r;
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
  {   // 정지선 S에 다가가는 직선에서 양쪽 차선으로 차선 대비 방향·옆 위치(원값)를 잰다 → 통과할 때 자세를 다시 잡음
    auto fit = [](const std::vector<Vec2> & pts, double & a, double & b) {   // y = a·x + b (가까운 직선 부분만: 앞 코너가 섞이지 않게)
      double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (const auto & q : pts) if (q.x >= 0.15 && q.x <= 0.8) { ++n; sx += q.x; sy += q.y; sxx += q.x * q.x; sxy += q.x * q.y; }
      const double den = n * sxx - sx * sx;
      if (n < 6 || std::abs(den) < 1e-9) return false;
      a = (n * sxy - sx * sy) / den; b = (sy - a * sx) / n;
      return true;
    };
    double al, bl, ar, br;
    // 추정 위치(오차가 쌓임)가 아니라 BEV에 정지선이 실제로 1 m 안에 보일 때만 잰다 → 확실히 S 앞 직선
    const Pose here = pose_at(s.t + calib_.latency);
    const Vec2 S = map_.stop_center();
    bool s_in_view = std::isfinite(s.stop_m) && s.stop_m > 0.2 && s.stop_m < 1.0;
    if (s_in_view) { const Vec2 w = to_world(here, {s.stop_m, 0.0}); s_in_view = std::hypot(w.x - S.x, w.y - S.y) <= cfg_.stop_gate_m; }
    const bool both = fit(s.left, al, bl) && fit(s.right, ar, br) && std::abs(al - ar) < 0.1 && bl - br > 0.3 && bl - br < 1.0;
    if (both) {   // 진단: BEV로 잰 차선 폭 (정답 0.65 m)
      const double wdt = (bl - br) * std::cos(std::atan(0.5 * (al + ar)));
      live_.lane_width = std::isfinite(live_.lane_width) ? live_.lane_width + 0.05 * (wdt - live_.lane_width) : wdt;
    }
    if (s_in_view && both) {
      lane_rel_yaw_ += std::atan(0.5 * (al + ar));   // 원값: 차선 기울기 각
      lane_rel_lat_ += 0.5 * (bl + br);              // 원값: 두 선 가운데의 차 기준 옆 위치
      ++lane_rel_n_;                                  // S에 다가가는 동안 평균
    }
  }
  // 매핑 (그 장면을 본 위치 = 촬영 시각 + 시간차). 정지선 S를 지나기 전(가속 구간)에는 하지 않음
  const Pose p = pose_at(s.t + calib_.latency);
  for (int side = 0; side < 2 && !live_.waiting_s; ++side) {
    const auto & pts = side == 0 ? s.left : s.right;
    const std::size_t stride = std::max<std::size_t>(1, pts.size() / 6);
    for (std::size_t k = 0; k < pts.size(); k += stride) {
      const Vec2 w = to_world(p, calib_.bev_point(pts[k]));
      const double d = std::min(kCap, map_.dist_side(side, run_.cw, w));
      lap_sq_ += d * d; ++lap_n_;
      if (new_pts_.size() < 1200) { new_pts_.push_back(w); new_side_.push_back(side); }
    }
  }
  // 정지선 통과 예측: 앞차축이 정지선에 닿는 누적 거리
  if (std::isfinite(s.stop_m) && s.stop_m > 0.15 && s.stop_m < cfg_.max_range_m) {
    const Pose seen = pose_at(s.t + calib_.latency);   // 그 장면을 본 위치
    const Vec2 w = to_world(seen, {s.stop_m, 0.0}), S = map_.stop_center();
    live_.stop_seen_m = s.stop_m;
    live_.stop_near_s = std::hypot(w.x - S.x, w.y - S.y) <= cfg_.stop_gate_m;
    if (!live_.stop_near_s) {   // 지도상 정지선 S가 아닌 곳: 오인식 (코너의 휜 선 등)
      if (!last_rejected_) ++live_.rejected_far;
      last_rejected_ = true;
    } else {
      last_rejected_ = false;
      const double d_cross = seen.dist + s.stop_m;   // 앞차축이 정지선에 닿는 누적 거리. 가까이서 본 최신 관측이 가장 정확
      if (stop_n_ > 0 && std::abs(d_cross - stop_cand_) < cfg_.stop_tolerance_m) { stop_cand_ = d_cross; ++stop_n_; }
      else { stop_cand_ = d_cross; stop_n_ = 1; }
      if (stop_n_ >= cfg_.stop_confirm) armed_dist_ = stop_cand_;
    }
  } else {
    live_.stop_seen_m = std::numeric_limits<double>::quiet_NaN();
    last_rejected_ = false;
  }
  if (!live_.waiting_s) run_.lanes.push_back(std::move(s));
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
