// Copied from src/cone_drive_test/tools/fit_planner.cpp (1113ML) — auto_control로 옮김
// 수동 주행 녹화로 회피 계획 파라미터 맞추기: 계획 경로가 사람이 실제로 달린 경로와 가장 비슷해지는 값을 찾는다.
//
//   fit_planner [--frames-every N] [--fit] <녹화 폴더>...
//
// 녹화 프레임마다 그때의 차선·기억 콘·자세·속도로 계획기를 다시 돌리고, 그 순간 이후 사람이 달린 경로(odom, 앞차축)를
// 그 순간 차량 좌표로 옮겨 같은 주행 거리(0.3~2.0 m)끼리 비교한다. 콘이 앞 3 m 안에 있는 프레임만 쓴다.
// --fit: 아래 파라미터를 패턴 탐색(한 번에 하나씩 늘려 보고 줄여 보기, 좋아지면 채택, 안 되면 걸음 반으로)으로 맞춘다.
// 결과는 params.yaml 형식으로 출력. 계획기 코드는 실차와 같은 것(AvoidancePlanner)을 쓴다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <future>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "auto_control/drive/core/avoidance_planner.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/frenet_planner.hpp"

using namespace auto_control::drive;

namespace {

std::vector<Vec2> parse_pts(const std::string & s) {
  std::vector<Vec2> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ';')) {
    double x, y;
    if (std::sscanf(item.c_str(), "%lf %lf", &x, &y) == 2) out.push_back({x, y});
  }
  return out;
}

std::vector<std::string> split_csv(const std::string & line) {
  std::vector<std::string> v;
  std::stringstream ss(line);
  std::string c;
  while (std::getline(ss, c, ',')) v.push_back(c);
  if (!line.empty() && line.back() == ',') v.push_back("");
  return v;
}

double json_num(const std::string & text, const std::string & key, double def) {
  const auto p = text.find("\"" + key + "\":");
  return p == std::string::npos ? def : std::atof(text.c_str() + p + key.size() + 3);
}

struct Frame {
  double t{0}, speed{0}, yaw_rate{0};
  Pose2 pose;
  LaneObservation lane;
  std::vector<Vec2> cones;
};

struct Case {   // 비교에 쓰는 프레임 하나
  const Frame * f;
  std::vector<Vec2> human;   // 그 순간 차량 좌표에서 사람이 달린 경로 (앞차축)
  double wheelbase;
};

struct Run { std::string dir; std::vector<Frame> frames; double wheelbase{0.33}, length{0.5}, width{0.3}, overhang{0.12}, cone{0.13}; };

// 콘 관측(cone_obs.csv)을 카메라 기준으로 bearing_deg 돌려 콘 기억을 처음부터 다시 만들고, 프레임마다 그 시각의 확정 콘으로 바꾼다
// (검출 방향 치우침을 고쳤다면 계획기가 봤을 콘)
void rebuild_cones(Run & r, double bearing_deg, double cam_x = -0.09) {
  std::ifstream in(r.dir + "/cone_obs.csv");
  std::string line;
  if (!std::getline(in, line)) return;
  const auto head = split_csv(line);
  std::map<std::string, std::size_t> col;
  for (std::size_t i = 0; i < head.size(); ++i) col[head[i]] = i;
  struct Obs { double t; Pose2 pose; ConeObservation o; };
  std::vector<Obs> obs;
  const double a = bearing_deg * M_PI / 180.0, ca = std::cos(a), sa = std::sin(a);
  while (std::getline(in, line)) {
    const auto c = split_csv(line);
    if (c.size() < head.size() || std::stoi(c[col["class"]]) != 0) continue;
    const double lx = std::stod(c[col["local_x"]]) - cam_x, ly = std::stod(c[col["local_y"]]);
    Obs q;
    q.t = std::stod(c[col["t_rgb"]]);
    q.pose = {std::stod(c[col["pose_x"]]), std::stod(c[col["pose_y"]]), std::stod(c[col["pose_yaw"]])};
    q.o.center = {cam_x + ca * lx - sa * ly, sa * lx + ca * ly};
    q.o.range = std::hypot(lx, ly);
    obs.push_back(q);
  }
  ConeMemory mem{MemoryParams{}};
  const auto none = [](const std::vector<Vec2> &, const Pose2 &) { return std::vector<int>{}; };
  std::size_t k = 0;
  for (auto & f : r.frames) {
    while (k < obs.size() && obs[k].t <= f.t) {   // 같은 검출 프레임(같은 t)의 관측을 한 번에
      const double t = obs[k].t;
      const Pose2 pose = obs[k].pose;
      std::vector<ConeObservation> batch;
      while (k < obs.size() && obs[k].t == t) batch.push_back(obs[k++].o);
      mem.update(pose, t, batch, none);
    }
    f.cones = mem.confirmed_world();
  }
}

Run load(const std::string & dir) {
  Run r;
  r.dir = dir;
  std::ifstream m(dir + "/meta.json");
  std::string meta((std::istreambuf_iterator<char>(m)), std::istreambuf_iterator<char>());
  r.wheelbase = json_num(meta, "wheelbase", 0.33);
  r.length = json_num(meta, "vehicle_length", 0.5);
  r.width = json_num(meta, "vehicle_width", 0.3);
  r.overhang = json_num(meta, "rear_overhang", 0.12);
  r.cone = json_num(meta, "cone_size", 0.13);
  std::ifstream in(dir + "/frames.csv");
  std::string line;
  std::getline(in, line);
  const auto head = split_csv(line);
  std::map<std::string, std::size_t> col;
  for (std::size_t i = 0; i < head.size(); ++i) col[head[i]] = i;
  while (std::getline(in, line)) {
    const auto c = split_csv(line);
    if (c.size() < head.size()) continue;
    Frame f;
    f.t = std::stod(c[col["t"]]);
    f.pose = {std::stod(c[col["pose_x"]]), std::stod(c[col["pose_y"]]), std::stod(c[col["pose_yaw"]])};
    f.speed = std::stod(c[col["speed"]]);
    f.yaw_rate = std::stod(c[col["yaw_rate"]]);
    f.lane.left = parse_pts(c[col["left"]]);
    f.lane.right = parse_pts(c[col["right"]]);
    f.lane.path = parse_pts(c[col["path"]]);
    if (col.count("center")) f.lane.center = parse_pts(c[col["center"]]);
    f.lane.capture_time = f.t;
    f.lane.valid = f.lane.path.size() >= 4;
    f.cones = parse_pts(c[col["cones"]]);
    r.frames.push_back(std::move(f));
  }
  return r;
}

// 경로를 주행 거리 s에서 보간
bool at_s(const std::vector<Vec2> & p, double s, Vec2 * out) {
  double acc = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    const double d = std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    if (acc + d >= s && d > 1e-9) {
      const double u = (s - acc) / d;
      *out = {p[i - 1].x + u * (p[i].x - p[i - 1].x), p[i - 1].y + u * (p[i].y - p[i - 1].y)};
      return true;
    }
    acc += d;
  }
  return false;
}

struct Knob { const char * name; double PlannerParams::*field; double lo, hi, step; };

// 맞출 파라미터 (계획 경로 모양을 정하는 것). 앞의 것은 GUI 파라미터, 뒤의 것은 코드 안 가중치
const std::vector<Knob> & knobs() {
  static const std::vector<Knob> k = {
    {"plan.soft_clearance", &PlannerParams::soft_clearance, 0.02, 0.30, 0.03},
    {"plan.relaxed_soft_clearance", &PlannerParams::relaxed_soft_clearance, 0.01, 0.20, 0.02},
    {"plan.free_soft_clearance", &PlannerParams::free_soft_clearance, 0.02, 0.30, 0.03},
    // 최소 여유(안전 하한)는 2 cm 아래로 내리지 않는다: 맞춤이 계획 실패를 줄이려고 안전 여유를 깎는 쪽으로 가지 않게
    {"plan.hard_clearance", &PlannerParams::hard_clearance, 0.02, 0.06, 0.01},
    {"plan.relaxed_hard_clearance", &PlannerParams::relaxed_hard_clearance, 0.02, 0.05, 0.01},
    {"plan.free_hard_clearance", &PlannerParams::free_hard_clearance, 0.02, 0.06, 0.01},
    {"plan.free_max_excess", &PlannerParams::free_max_excess, 0.2, 1.0, 0.1},
    {"plan.s_max", &PlannerParams::s_max, 1.5, 3.5, 0.5},
    {"plan.w_lane_linear_free", &PlannerParams::w_lane_linear_free, 0.5, 400, 0.0},   // 탐색에서 고정 (분석용 --set)
    {"plan.l_max_lane", &PlannerParams::l_max_lane, 0.2, 0.6, 0.0},
    {"plan.width", &PlannerParams::width, 0.2, 0.4, 0.0},
    {"plan.lane_tolerance", &PlannerParams::lane_tolerance, 0.0, 0.10, 0.02},
    {"plan.lane_return_distance", &PlannerParams::lane_return_distance, 0.2, 3.0, 0.3},
    {"plan.w_obstacle", &PlannerParams::w_obstacle, 200, 40000, 2000},
    {"plan.w_offset", &PlannerParams::w_offset, 1, 200, 10},
    {"plan.w_slope", &PlannerParams::w_slope, 0.05, 20, 0.5},
    {"plan.w_curvature", &PlannerParams::w_curvature, 0.005, 1.0, 0.04},
  };
  return k;
}

struct Score {
  double rms{0}, fail{0};
  int n{0};
  std::map<std::string, int> modes;
  std::vector<double> plan_excess, human_excess;   // 프레임마다 앞 2 m 경로에서 차체가 차선 밖으로 나간 최대량 (m)
};

// 경로(차량 좌표)의 차체 차선 이탈 최대량: 차선 기준선(lane.path)에서의 옆 거리 + 차체 반폭 − 차선 반폭 (방향 기울기 무시)
double path_excess(const std::vector<Vec2> & path, const std::vector<Vec2> & center, double half_width, double lane_half) {
  double worst = -1;
  for (const auto & q : path) {
    if (q.x < 0 || q.x > 2.0) continue;
    double best = 1e9;
    for (std::size_t i = 1; i < center.size(); ++i) {
      const Vec2 a = center[i - 1], b = center[i];
      const double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
      const double u = L2 > 0 ? std::clamp(((q.x - a.x) * dx + (q.y - a.y) * dy) / L2, 0.0, 1.0) : 0.0;
      best = std::min(best, std::hypot(a.x + u * dx - q.x, a.y + u * dy - q.y));
    }
    worst = std::max(worst, best + half_width - lane_half);
  }
  return worst;
}

Score evaluate(const std::vector<Case> & cases, const PlannerParams & p, const ControlParams & c, int threads) {
  std::vector<std::future<Score>> parts;
  const std::size_t chunk = (cases.size() + threads - 1) / std::max(1, threads);
  for (int t = 0; t < threads; ++t) {
    parts.push_back(std::async(std::launch::async, [&, t] {
      Score s;
      double sq = 0;
      for (std::size_t i = t * chunk; i < std::min(cases.size(), (t + 1) * chunk); ++i) {
        const auto & cs = cases[i];
        AvoidancePlanner planner(p, c);   // 이전 경로 기억 없이
        const double steer = cs.f->speed > 0.2 ? std::atan(cs.wheelbase * cs.f->yaw_rate / cs.f->speed) : 0.0;
        const auto res = planner.plan(cs.f->lane, cs.f->cones, cs.f->pose, cs.f->speed, steer);
        s.modes[plan_mode_name(res.mode)]++;
        s.plan_excess.push_back(path_excess(res.path_local, cs.f->lane.path, p.width / 2, p.lane_half));
        s.human_excess.push_back(path_excess(cs.human, cs.f->lane.path, p.width / 2, p.lane_half));
        if (res.mode == PlanMode::Fallback || res.path_local.size() < 3) { s.fail += 1; sq += 0.30 * 0.30 * 18; s.n += 18; continue; }
        for (double q = 0.3; q <= 2.0 + 1e-9; q += 0.1) {
          Vec2 a, b;
          if (!at_s(res.path_local, q, &a) || !at_s(cs.human, q, &b)) continue;
          const double d = std::min(0.5, std::hypot(a.x - b.x, a.y - b.y));
          sq += d * d;
          ++s.n;
        }
      }
      s.rms = sq;
      return s;
    }));
  }
  Score total;
  double sq = 0;
  for (auto & f : parts) {
    auto s = f.get();
    sq += s.rms; total.n += s.n; total.fail += s.fail;
    total.plan_excess.insert(total.plan_excess.end(), s.plan_excess.begin(), s.plan_excess.end());
    total.human_excess.insert(total.human_excess.end(), s.human_excess.begin(), s.human_excess.end());
    for (auto & [k, v] : s.modes) total.modes[k] += v;
  }
  total.rms = total.n ? std::sqrt(sq / total.n) : 1e9;
  total.fail /= std::max<std::size_t>(1, cases.size());
  return total;
}

double quant(std::vector<double> v, double q) {
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, std::size_t(q * (v.size() - 1)))];
}

void print_score(const char * tag, const Score & s) {
  auto out = [](const std::vector<double> & v, double lim) { double n = 0; for (double x : v) n += x > lim; return 100 * n / std::max<std::size_t>(1, v.size()); };
  std::printf("%s 경로 차이 RMS %.1f cm, 계획 실패 %.0f%% | 차선 이탈: 계획 중앙값 %.1f cm·90%% %.1f cm·10 cm 넘음 %.0f%% (사람 %.1f·%.1f cm·%.0f%%) |",
              tag, s.rms * 100, s.fail * 100, quant(s.plan_excess, 0.5) * 100, quant(s.plan_excess, 0.9) * 100, out(s.plan_excess, 0.10),
              quant(s.human_excess, 0.5) * 100, quant(s.human_excess, 0.9) * 100, out(s.human_excess, 0.10));
  int tot = 0;
  for (auto & [k, v] : s.modes) tot += v;
  for (auto & [k, v] : s.modes) std::printf(" %s %d%%", k.c_str(), 100 * v / std::max(1, tot));
  std::printf("\n");
}

}  // namespace

int main(int argc, char ** argv) {
  int every = 3;
  bool fit = false;
  std::vector<std::string> dirs;
  std::map<std::string, double> set;
  double bearing = std::nan("");
  std::string params_yaml;   // 비우면 첫 녹화의 params.yaml (녹화 당시 값)
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--frames-every" && i + 1 < argc) every = std::max(1, std::atoi(argv[++i]));
    else if (a == "--fit") fit = true;
    else if (a.rfind("--bearing=", 0) == 0) bearing = std::atof(a.c_str() + 10);
    else if (a == "--params" && i + 1 < argc) params_yaml = argv[++i];
    else if (a.rfind("--set=", 0) == 0) { const auto e = a.find('=', 6); set[a.substr(6, e - 6)] = std::atof(a.c_str() + e + 1); }
    else dirs.push_back(a);
  }
  if (dirs.empty()) { std::fprintf(stderr, "usage: fit_planner [--fit] [--frames-every N] [--set=plan.x=v] <manual run dir>...\n"); return 2; }
  std::vector<Run> runs;
  for (const auto & d : dirs) runs.push_back(load(d));
  if (std::isfinite(bearing)) {
    for (auto & r : runs) rebuild_cones(r, bearing);
    std::printf("콘 기억을 관측으로 다시 만듦 (방향 %+.2f°)\n", bearing);
  }
  // 녹화 당시 파라미터에서 시작 (params.yaml의 plan.* 값)
  PlannerParams p;
  ControlParams c;
  {
    std::ifstream y(params_yaml.empty() ? dirs[0] + "/params.yaml" : params_yaml);
    std::string line;
    while (std::getline(y, line)) {
      for (const auto & k : knobs()) {
        const std::string key = std::string(k.name) + ":";
        const auto pos = line.find(key);
        if (pos != std::string::npos) p.*(k.field) = std::atof(line.c_str() + pos + key.size());
      }
    }
  }
  for (const auto & [name, v] : set)
    for (const auto & k : knobs()) if (name == k.name) p.*(k.field) = v;
  p.length = runs[0].length; p.width = runs[0].width; p.rear_overhang = runs[0].overhang; p.wheelbase = runs[0].wheelbase;
  p.cone_half = runs[0].cone / 2;
  if (set.count("plan.cone_half")) p.cone_half = set["plan.cone_half"];
  if (set.count("plan.width")) p.width = set["plan.width"];
  // 비교할 프레임: 앞 3 m 안에 콘, 이후 2.0 m 이상 사람 경로가 있는 프레임
  std::vector<Case> cases;
  for (const auto & r : runs) {
    for (std::size_t i = 0; i < r.frames.size(); i += every) {
      const auto & f = r.frames[i];
      if (!f.lane.valid || f.speed < 0.2) continue;
      bool near = false;
      for (const auto & w : f.cones) { const auto l = to_local(w, f.pose); if (l.x > -0.3 && l.x < 3.0 && std::abs(l.y) < 1.0) near = true; }
      if (!near) continue;
      Case cs{&f, {}, r.wheelbase};
      cs.human.push_back({0, 0});
      double len = 0;
      for (std::size_t j = i + 1; j < r.frames.size() && len < 2.2; ++j) {
        const Vec2 q = to_local({r.frames[j].pose.x, r.frames[j].pose.y}, f.pose);
        len += std::hypot(q.x - cs.human.back().x, q.y - cs.human.back().y);
        cs.human.push_back(q);
      }
      if (len >= 2.0) cases.push_back(std::move(cs));
    }
  }
  std::printf("녹화 %zu개, 비교 프레임 %zu개 (콘이 앞 3 m 안, %d프레임마다)\n", runs.size(), cases.size(), every);
  const int threads = std::max(1, int(std::thread::hardware_concurrency()));
  auto best = evaluate(cases, p, c, threads);
  print_score("녹화 당시 파라미터:", best);
  if (fit) {
    std::vector<double> step;
    for (const auto & k : knobs()) step.push_back(k.step);
    for (int round = 0; round < 4; ++round) {
      bool any = false;
      for (std::size_t i = 0; i < knobs().size(); ++i) {
        const auto & k = knobs()[i];
        for (const double dir : {+1.0, -1.0}) {
          PlannerParams q = p;
          q.*(k.field) = std::clamp(p.*(k.field) + dir * step[i], k.lo, k.hi);
          if (q.*(k.field) == p.*(k.field)) continue;
          // 순서 유지: 최소 ≤ 권장
          if (q.hard_clearance > q.soft_clearance || q.relaxed_hard_clearance > q.relaxed_soft_clearance || q.free_hard_clearance > q.free_soft_clearance) continue;
          const auto s = evaluate(cases, q, c, threads);
          if (s.rms < best.rms - 1e-4) {
            p = q; best = s; any = true;
            std::printf("  %-30s → %.3f  (RMS %.1f cm)\n", k.name, p.*(k.field), s.rms * 100);
            break;
          }
        }
      }
      if (!any) for (auto & st : step) st /= 2;
    }
    print_score("맞춘 파라미터:", best);
  }
  std::printf("\n# 결과 (params.yaml 형식)\n");
  for (const auto & k : knobs()) std::printf("    %s: %.4g\n", k.name, p.*(k.field));
  return 0;
}
