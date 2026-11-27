#include "check.hpp"
#include "auto_control/drive/core/params.hpp"
#include "auto_control/drive/core/settings.hpp"
#include "auto_control/drive/core/stage_stats.hpp"
#include "auto_control/drive/core/system_monitor.hpp"
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

using namespace auto_control::drive;

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
  const auto yaml = r.to_yaml("auto_drive");
  CHECK(yaml.rfind("/**/auto_drive:", 0) == 0);   // I4: namespace 아래 노드에도 적용되는 키
  CHECK(yaml.find("ros__parameters:") != std::string::npos);
  std::istringstream in(yaml);
  std::map<std::string, double> parsed;
  std::string line;
  while (std::getline(in, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos || line.find("ros__parameters") != std::string::npos || line.rfind("/**/auto_drive", 0) == 0) continue;
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

TEST(tunnel_tuning_validates_and_saves_detector_and_live_tunnel_settings) {
  ParamRegistry r;
  register_params(r);
  CHECK(r.get("tcn.threshold") == 0.5);
  CHECK(r.get("tunnel.validation_mode") == 1);
  CHECK(!r.apply({{"tcn.threshold", 0.65}, {"tcn.max_gap_seconds", 0}}).first);
  CHECK(r.get("tcn.threshold") == 0.5);
  CHECK(!r.apply({{"tunnel.validation_mode", 0.5}}).first);
  CHECK(r.apply({{"tcn.threshold", 0.65}, {"tunnel.validation_mode", 0}}).first);
  const auto yaml = r.to_yaml("auto_drive");
  CHECK(yaml.find("tcn.threshold: 0.65") != std::string::npos);
  CHECK(yaml.find("tunnel.validation_mode: 0.0") != std::string::npos);
}

TEST(restart_yaml_validates_changes_without_changing_running_parameters) {
  ParamRegistry r;
  register_params(r);
  const auto active = r.to_yaml("auto_drive");
  const auto version = r.version();
  const auto [ok, yaml] = r.yaml_with_changes("auto_drive",
    {{"rgb_cam.iso", 500}, {"ir_lane.result_padding_px", 25}});
  CHECK(ok);
  CHECK(yaml.find("rgb_cam.iso: 500.0") != std::string::npos);
  CHECK(yaml.find("ir_lane.result_padding_px: 25.0") != std::string::npos);
  CHECK(r.to_yaml("auto_drive") == active);
  CHECK(r.version() == version);
  const auto [invalid, reason] = r.yaml_with_changes("auto_drive",
    {{"rgb_cam.iso", 500}, {"ir_lane.result_padding_px", 25.5}});
  CHECK(!invalid);
  CHECK(reason.find("ir_lane.result_padding_px") != std::string::npos);
  CHECK(r.to_yaml("auto_drive") == active);
}

TEST(rgb_camera_parameters_are_live_and_independent_of_ir) {
  using Mode = bev_handoff::RgbExposureMode;
  ParamRegistry r;
  register_params(r);
  const auto defaults = settings_from(r);
  CHECK(defaults.rgb_camera.mode == Mode::automatic);
  CHECK(defaults.rgb_camera.exposure_us == 8000);
  CHECK(defaults.rgb_camera.iso == 400);
  CHECK(!r.apply({{"rgb_cam.auto_exposure", .5}}).first);
  CHECK(!r.apply({{"rgb_cam.exposure_us", 15000}}).first);
  CHECK(!r.apply({{"rgb_cam.iso", 1700}}).first);
  CHECK(r.apply({{"rgb_cam.auto_exposure", 0}, {"rgb_cam.exposure_us", 6000}, {"rgb_cam.iso", 600}}).first);
  const auto updated = settings_from(r);
  CHECK(updated.rgb_camera.mode == Mode::manual);
  CHECK(updated.rgb_camera.exposure_us == 6000);
  CHECK(updated.rgb_camera.iso == 600);
  CHECK(updated.ir_camera.inside.iso == defaults.ir_camera.inside.iso);
  CHECK(r.to_yaml("auto_drive").find("rgb_cam.iso: 600.0") != std::string::npos);
  CHECK(r.apply({{"rgb_cam.auto_exposure", 2}}).first);
  CHECK(settings_from(r).rgb_camera.mode == Mode::fixed_iso);
  CHECK(r.apply({{"rgb_cam.auto_exposure", 1}}).first);
  CHECK(settings_from(r).rgb_camera.mode == Mode::automatic);
}

TEST(tunnel_ir_resolution_uses_supported_choices) {
  ParamRegistry r;
  register_params(r);
  const auto defaults = settings_from(r).ir_camera;
  CHECK(!defaults.use_400p_ir_output);
  CHECK(defaults.inside.exposure_us == 20000);
  CHECK(defaults.inside.iso == 500);
  CHECK(defaults.inside.dot == 0.0);
  CHECK(defaults.inside.flood == 0.7);
  CHECK(r.to_json().find("\"options\":[{\"value\":0") != std::string::npos);

  CHECK(!r.apply({{"ir_cam.inside_resolution", 200}, {"ir_cam.inside_iso", 600}}).first);
  CHECK(settings_from(r).ir_camera.inside.iso == 500);
  CHECK(r.apply({{"ir_cam.inside_resolution", 400}, {"ir_cam.inside_iso", 600},
                 {"ir_cam.inside_exposure_us", 18000}, {"ir_cam.inside_flood", 0.25},
                 {"ir_cam.inside_dot", 0.125}}).first);
  const auto updated = settings_from(r).ir_camera;
  CHECK(updated.use_400p_ir_output);
  CHECK(updated.inside.iso == 600);
  CHECK(updated.inside.exposure_us == 18000);
  CHECK_NEAR(updated.inside.flood, 0.25, 1e-12);
  CHECK_NEAR(updated.inside.dot, 0.125, 1e-12);
  CHECK(r.to_yaml("auto_drive").find("ir_cam.inside_resolution: 400.0") != std::string::npos);

  CHECK(r.apply({{"ir_cam.inside_resolution", 0}}).first);
  CHECK(!settings_from(r).ir_camera.use_400p_ir_output);
}

TEST(lane_tuning_uses_independent_rgb_and_ir_profiles) {
  ParamRegistry r;
  register_params(r);
  const auto defaults = settings_from(r);
  CHECK(defaults.rgb_lane.bridge_centerline);
  CHECK(defaults.ir_lane.bridge_centerline);
  CHECK_NEAR(defaults.rgb_lane.left_mask_threshold, .8, 1e-12);
  CHECK_NEAR(defaults.rgb_lane.right_mask_threshold, .8, 1e-12);
  CHECK_NEAR(defaults.rgb_lane.stop_line_mask_threshold, .8, 1e-12);
  CHECK(defaults.rgb_lane.connection_min_component_area_px == 30);
  CHECK(defaults.rgb_lane.connection_max_gap_px == 90.);
  CHECK(defaults.rgb_lane.corner_outer_weight == .85);
  CHECK(defaults.rgb_lane.corner_outward_offset_m == 0.);
  CHECK(defaults.rgb_lane.stop_line_min_pixels == 30);
  CHECK(!defaults.ir_lane.stop_line_distance_enabled);
  CHECK_NEAR(defaults.ir_lane.left_mask_threshold, .8, 1e-12);
  CHECK_NEAR(defaults.ir_lane.right_mask_threshold, .8, 1e-12);
  CHECK(r.get("lane.model") == 2.);
  CHECK(!r.apply({{"lane.model", 4.}}).first);
  CHECK(!r.apply({{"lane.model", 2.5}}).first);
  CHECK(!r.apply({{"lane.corner_outer_full_turn_deg", 7.}, {"ir_lane.mask_threshold", .6}}).first);
  CHECK(!r.apply({{"ir_lane.left_mask_threshold", -.5}}).first);
  CHECK(!r.apply({{"lane.result_padding_px", 2.5}}).first);
  CHECK(!r.apply({{"ir_lane.engine_precision", 1.}}).first);   // replaced by the shared lane.model
  CHECK(!r.apply({{"lane.engine_precision", 1.}}).first);
  CHECK(r.apply({{"lane.model", 2.}}).first);
  CHECK(r.get("lane.model") == 2.);
  CHECK(r.apply({{"ir_lane.mask_threshold", .6}, {"ir_lane.right_mask_threshold", .72},
    {"ir_lane.bridge_centerline", 0.}, {"lane.left_mask_threshold", .75}}).first);
  const auto tuned = settings_from(r);
  CHECK_NEAR(tuned.ir_lane.left_mask_threshold, .6, 1e-12);
  CHECK_NEAR(tuned.ir_lane.right_mask_threshold, .72, 1e-12);
  CHECK_NEAR(tuned.rgb_lane.left_mask_threshold, .75, 1e-12);
  CHECK_NEAR(tuned.rgb_lane.right_mask_threshold, .8, 1e-12);
  CHECK(tuned.rgb_lane.bridge_centerline);
  CHECK(!tuned.ir_lane.bridge_centerline);
  for (const auto & spec : r.list()) {
    if (spec.name.rfind("lane.", 0) == 0 || spec.name.rfind("ir_lane.", 0) == 0)
      CHECK(spec.group == "lane");
  }
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
