#include "check.hpp"
#include "cone_drive_test/core/params.hpp"
#include "cone_drive_test/core/stage_stats.hpp"
#include "cone_drive_test/core/system_monitor.hpp"
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

using namespace cone_drive_test;

static void fill(ParamRegistry & r) {
  r.add({"drive.minimum_speed_mps", "driving", "속도", "최소 속도", "m/s", 0.0, 2.0, 1.0, true});
  r.add({"drive.maximum_speed_mps", "driving", "속도", "최대 속도", "m/s", 0.0, 3.0, 1.8, true});
  r.add({"yolo.con_threshold", "obstacle", "YOLO", "콘 임계값", "", 0.0, 1.0, 0.7, true});
  r.add_order_constraint("drive.minimum_speed_mps", "drive.maximum_speed_mps");
}

TEST(apply_rejects_invalid_atomically) {   // Review Focus 4
  ParamRegistry r;
  fill(r);
  const auto v0 = r.version();
  const std::map<std::string, double> bad[] = {
    {{"yolo.con_threshold", 0.5}, {"drive.maximum_speed_mps", std::nan("")}},
    {{"yolo.con_threshold", 0.5}, {"drive.maximum_speed_mps", 3.5}},
    {{"drive.minimum_speed_mps", 2.0}, {"drive.maximum_speed_mps", 1.5}},
    {{"no.such_param", 1.0}},
  };
  const char * names[] = {"drive.maximum_speed_mps", "drive.maximum_speed_mps", "drive.minimum_speed_mps", "no.such_param"};
  for (int k = 0; k < 4; ++k) {
    const auto res = r.apply(bad[k]);
    CHECK(!res.first);
    CHECK(res.second.find(names[k]) != std::string::npos);
    CHECK_NEAR(r.get("yolo.con_threshold"), 0.7, 1e-12);
    CHECK_NEAR(r.get("drive.minimum_speed_mps"), 1.0, 1e-12);
    CHECK_NEAR(r.get("drive.maximum_speed_mps"), 1.8, 1e-12);
    CHECK(r.version() == v0);
  }
  CHECK(r.apply({{"yolo.con_threshold", 0.5}, {"drive.maximum_speed_mps", 1.2}}).first);
  CHECK_NEAR(r.get("yolo.con_threshold"), 0.5, 1e-12);
  CHECK(r.version() == v0 + 1);
}

TEST(yaml_roundtrip) {
  ParamRegistry r;
  fill(r);
  r.apply({{"drive.maximum_speed_mps", 1.25}});
  const auto yaml = r.to_yaml("cone_drive_test");
  CHECK(yaml.rfind("/**/cone_drive_test:", 0) == 0);   // I4: namespace 아래 노드에도 적용되는 키
  CHECK(yaml.find("ros__parameters:") != std::string::npos);
  std::istringstream in(yaml);
  std::map<std::string, double> parsed;
  std::string line;
  while (std::getline(in, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos || line.find("ros__parameters") != std::string::npos || line.rfind("/**/cone_drive_test", 0) == 0) continue;
    std::string key = line.substr(0, colon);
    key.erase(0, key.find_first_not_of(' '));
    parsed[key] = std::stod(line.substr(colon + 1));
  }
  for (const auto & p : r.list()) CHECK_NEAR(parsed.at(p.name), p.value, 1e-9);
}

TEST(params_json_lists_specs) {
  ParamRegistry r;
  fill(r);
  const auto j = r.to_json();
  CHECK(j.find("\"name\":\"drive.maximum_speed_mps\"") != std::string::npos);
  CHECK(j.find("\"group\":\"obstacle\"") != std::string::npos);
  CHECK(j.find("\"live\":true") != std::string::npos);
}

TEST(stage_stats_window) {
  StageStats s(1.0);
  for (int k = 0; k < 30; ++k) s.record("lane_infer", 10.0 + k * 0.03, 10.0 + k * 0.03 + 0.010);
  s.record("old", 5.0, 5.5);   // 1 s 창 밖
  const auto rows = s.snapshot(11.0);
  CHECK(rows.size() == 1);
  CHECK(rows[0].stage == "lane_infer");
  CHECK_NEAR(rows[0].avg_ms, 10.0, 1e-6);
  CHECK_NEAR(rows[0].fps, 30.0, 1e-3);   // 1초에 30번 처리 (1000/10ms=100이 아님)
}

TEST(system_monitor_reads_fake_proc) {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() / "cdt_sysmon";
  fs::remove_all(root);
  fs::create_directories(root / "proc/self");
  fs::create_directories(root / "sys/devices/platform/gpu.0");
  auto write = [](const fs::path & p, const std::string & s) { std::ofstream(p) << s; };
  write(root / "proc/stat", "cpu  100 0 100 800 0 0 0 0 0 0\n");
  write(root / "proc/meminfo", "MemTotal:       1000 kB\nMemFree: 100 kB\nMemAvailable:    500 kB\n");
  write(root / "proc/self/stat", "1 (x) S 0 0 0 0 0 0 0 0 0 0 10 10 0 0\n");
  write(root / "sys/devices/platform/gpu.0/load", "500\n");
  SystemMonitor mon((root / "proc").string(), (root / "sys").string());
  auto u = mon.sample(0.0);
  CHECK_NEAR(u.mem_percent, 50.0, 1e-9);
  CHECK(u.gpu_percent.has_value());
  CHECK_NEAR(*u.gpu_percent, 50.0, 1e-9);
  write(root / "proc/stat", "cpu  150 0 150 900 0 0 0 0 0 0\n");   // busy +100, idle +100
  u = mon.sample(1.0);
  CHECK_NEAR(u.cpu_percent, 50.0, 1e-9);
  fs::remove(root / "sys/devices/platform/gpu.0/load");
  CHECK(!mon.sample(2.0).gpu_percent.has_value());
}
