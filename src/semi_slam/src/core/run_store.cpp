#include "semi_slam/core/run_store.hpp"
#include "semi_slam/core/json_writer.hpp"
#include "semi_slam/core/mini_json.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace semi_slam {

namespace fs = std::filesystem;

namespace {
std::string now_string(const char * fmt) {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[64];
  std::strftime(buf, sizeof(buf), fmt, &tm);
  return buf;
}
std::string pose_json(const Pose & p) {
  JsonWriter w;
  w.begin_object().key("t").value(p.t, 6).key("x").value(p.x, 5).key("y").value(p.y, 5).key("yaw").value(p.yaw, 6).end_object();
  return w.str();
}
std::string read_file(const std::string & path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
double num_or(const JsonValue * j, const char * k, double d) {
  const auto * v = j ? j->get(k) : nullptr;
  return v && v->kind == JsonValue::Number ? v->num : d;
}
std::string str_or(const JsonValue * j, const char * k) {
  const auto * v = j ? j->get(k) : nullptr;
  return v && v->kind == JsonValue::String ? v->str : std::string();
}
std::string pts(const std::vector<Vec2> & v) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(4);
  for (std::size_t i = 0; i < v.size(); ++i) o << (i ? ";" : "") << v[i].x << ' ' << v[i].y;
  return o.str();
}
std::vector<Vec2> parse_pts(const std::string & s) {
  std::vector<Vec2> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ';')) { std::stringstream it(item); Vec2 p; if (it >> p.x >> p.y) out.push_back(p); }
  return out;
}
}  // namespace

std::string make_run_dir(const std::string & data_dir, double v) {
  char suffix[32];
  std::snprintf(suffix, sizeof(suffix), "_v%.2f", v);
  std::string dir = data_dir + "/runs/" + now_string("%Y-%m-%d_%H%M%S") + suffix;
  fs::create_directories(dir);
  return dir;
}

void write_run(const std::string & dir, const RunData & run, const OdomCalib & calib, const std::vector<LapMetrics> & laps,
               const std::vector<std::pair<int, double>> & corr, const std::vector<Pose> & traj) {
  fs::create_directories(dir);
  {
    std::ofstream o(dir + "/meta.json");
    o << "{\"created\":\"" << now_string("%Y-%m-%d %H:%M:%S") << "\",\"target_speed\":" << run.target_speed
      << ",\"laps_target\":" << run.laps_target << ",\"laps_done\":" << run.crossing_t.size()
      << ",\"stopline_correction\":" << (run.stopline_correction ? "true" : "false")
      << ",\"direction\":\"" << (run.cw ? "cw" : "ccw") << "\""
      << ",\"params\":{";
    bool first = true;
    for (const auto & [k, v] : run.params) { o << (first ? "" : ",") << "\"" << k << "\":" << v; first = false; }
    o << "}"
      << ",\"start_rel_yaw\":" << (std::isfinite(run.start_rel.x) ? std::to_string(run.start_rel.x) : "null") << ",\"start_rel_lat\":" << run.start_rel.y << ",\"crossing_latency\":" << run.crossing_latency << ",\"gyro_bias0\":" << std::setprecision(9) << run.gyro_bias0 << ",\"start\":" << pose_json(run.start)
      << ",\"calib_used\":" << calib.to_json() << "}\n";
  }
  { std::ofstream o(dir + "/speed.csv"); o << "t,v\n" << std::setprecision(10); for (const auto & s : run.speed) o << s.t << ',' << s.v << '\n'; }
  { std::ofstream o(dir + "/gyro.csv"); o << "t,w\n" << std::setprecision(10); for (const auto & g : run.gyro) o << g.t << ',' << g.w << '\n'; }
  {
    std::ofstream o(dir + "/lanes.csv");
    o << "t,stop_m,left,right\n" << std::setprecision(10);
    for (const auto & l : run.lanes) o << l.t << ',' << (std::isfinite(l.stop_m) ? std::to_string(l.stop_m) : "nan") << ',' << pts(l.left) << ',' << pts(l.right) << '\n';
  }
  {
    std::ofstream o(dir + "/crossings.csv");
    o << "lap,t,rel_yaw,rel_lat\n" << std::setprecision(10);
    for (std::size_t k = 0; k < run.crossing_t.size(); ++k) {
      const Vec2 a = k < run.anchor_rel.size() ? run.anchor_rel[k] : Vec2{std::nan(""), 0};
      o << k + 1 << ',' << run.crossing_t[k] << ',' << (std::isfinite(a.x) ? std::to_string(a.x) : "nan") << ',' << a.y << '\n';
    }
  }
  {
    std::ofstream o(dir + "/laps.csv");
    o << "lap,dist_m,s_err_m,turn_deg,lat_mean_m,free_along_err_m,free_heading_err_deg,length_err,rms_m,points\n";
    for (const auto & m : laps)
      o << m.lap << ',' << m.dist_m << ',' << m.s_err_m << ',' << m.turn_deg << ',' << m.lat_mean_m << ',' << m.along_err_m << ','
        << m.heading_err_deg << ',' << m.length_err << ',' << m.rms_m << ',' << m.points << '\n';
  }
  { std::ofstream o(dir + "/corrections.csv"); o << "lap,shift_m\n"; for (const auto & [lap, s] : corr) o << lap << ',' << s << '\n'; }
  { std::ofstream o(dir + "/trajectory.csv"); o << "t,x,y,yaw,dist\n" << std::setprecision(10); for (const auto & p : traj) o << p.t << ',' << p.x << ',' << p.y << ',' << p.yaw << ',' << p.dist << '\n'; }
}

void write_calibration_result(const std::string & dir, const CalibResult & r) {
  std::ofstream o(dir + "/calibration.json");
  JsonWriter w;
  auto ev = [&](const char * k, const Evaluation & e) {
    w.key(k).begin_object().key("rms_m").value(e.rms_m).key("accuracy").value(e.accuracy).key("stop_rms_m").value(e.stop_rms_m)
     .key("points").value((long long)e.points).key("cost").value(e.cost, 6).end_object();
  };
  const auto & e = r.est;
  w.begin_object().key("applied").value(r.applied).key("gyro_used").value(r.gyro_used).key("wheel_used").value(r.wheel_used)
   .key("message").value(r.message);
  w.key("estimate").begin_object().key("ok").value(e.ok).key("message").value(e.message).key("laps").value((long long)e.laps)
   .key("gyro_scale").value(e.gyro_scale, 6).key("wheel_scale").value(e.wheel_scale, 6).key("raw_turn_deg_per_lap").value(e.raw_turn_deg_per_lap, 3)
   .key("raw_dist_per_lap").value(e.raw_dist_per_lap, 4).key("ref_length").value(e.ref_length, 4)
   .key("bev_yaw_residual_deg").value(e.bev_yaw_residual_deg, 2).key("straight_angle_deg").value(e.straight_angle_deg, 2)
   .key("straight_slip_deg").value(e.straight_slip_deg, 2).key("straights").value((long long)e.straights).end_object();
  ev("before", r.before);
  ev("after", r.after);
  w.end_object();
  o << "{\"result\":" << w.str() << ",\"calib\":" << r.calib.to_json() << "}\n";
}

RunData load_run(const std::string & dir) {
  RunData r;
  const auto meta = parse_json(read_file(dir + "/meta.json"));
  if (meta) {
    r.target_speed = num_or(&*meta, "target_speed", 0);
    r.laps_target = int(num_or(&*meta, "laps_target", 0));
    r.gyro_bias0 = num_or(&*meta, "gyro_bias0", 0);
    r.crossing_latency = num_or(&*meta, "crossing_latency", 0);
    const auto * sy = meta->get("start_rel_yaw");
    if (sy && sy->kind == JsonValue::Number) r.start_rel = {sy->num, num_or(&*meta, "start_rel_lat", 0)};
    const auto * sc = meta->get("stopline_correction");
    r.stopline_correction = !sc || sc->b;
    const auto * dr = meta->get("direction");   // 없으면 반시계 (예전 기록)
    r.cw = dr && dr->kind == JsonValue::String && dr->str == "cw";
    const auto * st = meta->get("start");
    r.start = {num_or(st, "t", 0), num_or(st, "x", 0), num_or(st, "y", 0), num_or(st, "yaw", 0), 0};
  }
  auto rows = [&](const std::string & file, const std::function<void(const std::vector<std::string> &)> & fn) {
    std::ifstream in(dir + "/" + file);
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
      std::vector<std::string> cols;
      std::stringstream ss(line);
      std::string c;
      while (std::getline(ss, c, ',')) cols.push_back(c);
      if (!cols.empty()) fn(cols);
    }
  };
  rows("speed.csv", [&](const auto & c) { if (c.size() >= 2) r.speed.push_back({std::stod(c[0]), std::stod(c[1])}); });
  rows("gyro.csv", [&](const auto & c) { if (c.size() >= 2) r.gyro.push_back({std::stod(c[0]), std::stod(c[1])}); });
  rows("crossings.csv", [&](const auto & c) {
    if (c.size() < 2) return;
    r.crossing_t.push_back(std::stod(c[1]));
    r.anchor_rel.push_back(c.size() >= 4 && c[2] != "nan" ? Vec2{std::stod(c[2]), std::stod(c[3])} : Vec2{std::nan(""), 0});
  });
  rows("lanes.csv", [&](const auto & c) {
    if (c.size() < 2) return;
    LaneSample s;
    s.t = std::stod(c[0]);
    s.stop_m = c[1] == "nan" ? std::numeric_limits<double>::quiet_NaN() : std::stod(c[1]);
    if (c.size() > 2) s.left = parse_pts(c[2]);
    if (c.size() > 3) s.right = parse_pts(c[3]);
    r.lanes.push_back(std::move(s));
  });
  return r;
}

CalibStore::CalibStore(std::string data_dir) : dir_(std::move(data_dir)) { fs::create_directories(dir_); }

OdomCalib CalibStore::load() const {
  const std::string text = read_file(dir_ + "/calibration.json");
  return text.empty() ? OdomCalib{} : OdomCalib::from_json(text);
}

void CalibStore::save(const OdomCalib & c) const {
  { std::ofstream o(dir_ + "/calibration.json"); o << c.to_json() << '\n'; }
  std::ofstream y(dir_ + "/odometry_calibration.yaml");
  y << "# semi_slam 오도메트리 보정 (" << now_string("%Y-%m-%d %H:%M:%S") << ")\n"
    << "# 속도 v = ERPM 속도 × 거리 배율(속도별 보간), 요레이트 = 자이로 배율 × (자이로 − 정지 영점)\n"
    << "# 정지선 S → S 한 바퀴 = 360°, 중앙선 길이 기준. 누적 바퀴 수: 자이로 " << c.gyro_laps << "\n"
    << "odometry:\n"
    << "  gyro_scale: " << c.gyro_scale << "\n"
    << "  gyro_bias_rad_s: " << c.gyro_bias << "\n"
    << "  camera_latency_sec: " << c.latency << "\n"
    << "  wheel_scale_by_speed_cm_s:\n";
  for (const auto & [k, s] : c.wheel_scale) y << "    \"" << k << "\": " << s << "\n";
}

void CalibStore::record(const CalibResult & r, const std::string & run_dir, const OdomCalib & previous, double speed, int laps, bool applied) const {
  std::ofstream o(dir_ + "/calibration_history.jsonl", std::ios::app);
  JsonWriter w;
  w.begin_object().key("time").value(now_string("%Y-%m-%d %H:%M:%S")).key("run").value(fs::path(run_dir).filename().string())
   .key("action").value(applied ? "applied" : "skipped").key("speed").value(speed, 2).key("laps").value((long long)laps)
   .key("gyro_est").value(r.est.gyro_scale, 5).key("wheel_est").value(r.est.wheel_scale, 5).key("yaw_residual").value(r.est.bev_yaw_residual_deg, 2)
   .key("rms_before").value(r.before.rms_m).key("rms_after").value(r.after.rms_m).key("accuracy_after").value(r.after.accuracy).end_object();
  std::string line = w.str();
  line.pop_back();   // 객체 닫기 전에 보정 값 두 개를 붙인다
  o << line << ",\"calib\":" << r.calib.to_json() << ",\"previous\":" << previous.to_json() << "}\n";
}

bool CalibStore::revert(OdomCalib * out) const {
  std::ifstream in(dir_ + "/calibration_history.jsonl");
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) if (!line.empty()) lines.push_back(line);
  // 되돌리기도 이력에 남기므로, 아직 되돌리지 않은 가장 최근 적용을 찾는다
  int skip = 0;
  for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
    const auto j = parse_json(*it);
    if (!j) continue;
    const std::string action = str_or(&*j, "action");
    if (action == "reverted") { ++skip; continue; }
    if (action != "applied") continue;
    if (skip > 0) { --skip; continue; }
    const auto * prev = j->get("previous");
    if (!prev) return false;
    // previous 객체를 다시 문자열로 만들기보다, 원문에서 잘라낸다
    const auto pos = it->find("\"previous\":");
    const std::string prev_text = it->substr(pos + 11, it->size() - pos - 12);
    *out = OdomCalib::from_json(prev_text);
    save(*out);
    std::ofstream o(dir_ + "/calibration_history.jsonl", std::ios::app);
    o << "{\"time\":\"" << now_string("%Y-%m-%d %H:%M:%S") << "\",\"run\":\"" << str_or(&*j, "run") << "\",\"action\":\"reverted\"}\n";
    return true;
  }
  return false;
}

std::string CalibStore::reset() const {
  const std::string backup = dir_ + "/backup_" + now_string("%Y-%m-%d_%H%M%S");
  fs::create_directories(backup);
  for (const char * f : {"calibration.json", "calibration_history.jsonl", "odometry_calibration.yaml"}) {
    std::error_code ec;
    if (fs::exists(dir_ + "/" + f)) fs::rename(dir_ + "/" + f, backup + "/" + f, ec);
  }
  save(OdomCalib{});
  return backup;
}

std::vector<CalibHistoryEntry> CalibStore::history(std::size_t last_n) const {
  std::ifstream in(dir_ + "/calibration_history.jsonl");
  std::vector<CalibHistoryEntry> all;
  std::string line;
  while (std::getline(in, line)) {
    const auto j = parse_json(line);
    if (!j) continue;
    CalibHistoryEntry e;
    e.time = str_or(&*j, "time"); e.run = str_or(&*j, "run"); e.action = str_or(&*j, "action");
    e.speed = num_or(&*j, "speed", 0); e.laps = int(num_or(&*j, "laps", 0));
    e.gyro_est = num_or(&*j, "gyro_est", std::nan(""));
    e.wheel_est = num_or(&*j, "wheel_est", std::nan(""));
    e.yaw_residual = num_or(&*j, "yaw_residual", std::nan(""));
    e.rms_before = num_or(&*j, "rms_before", 0); e.rms_after = num_or(&*j, "rms_after", 0); e.accuracy_after = num_or(&*j, "accuracy_after", 0);
    all.push_back(e);
  }
  if (all.size() > last_n) all.erase(all.begin(), all.end() - long(last_n));
  return all;
}

}  // namespace semi_slam
