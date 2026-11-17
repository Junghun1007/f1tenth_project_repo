#include "check.hpp"
#include "cone_drive_test/core/telemetry.hpp"
#include <nlohmann/json.hpp>

using namespace cone_drive_test;

static TelemetryFrame sample() {
  TelemetryFrame f;
  f.t = 1.5; f.state = "DRIVING"; f.mode = "drive"; f.reason = "say \"hi\"\n";
  f.readiness = {true, true, true, true, true, true, true};
  f.inspection.attitude_status = "PASS"; f.inspection.height_m = 0.2;
  f.inspection.models.push_back({"lane", "READY", "ready", "", 2.0});
  for (int k = 0; k < 200; ++k) f.lane_left.push_back({k * 0.015, 0.33});
  f.lane_right = {{0, -0.33}, {3, -0.33}};
  f.plan_path = {{0, 0}, {1, 0.1}};
  f.plan_mode = "lane"; f.rollout_ok = true;
  f.detections.push_back({"con", 0.9f, 1.2, 0.1, 0.13, 0.13, 0.0});
  f.memory.push_back({1.2, 0.1, true, 7});
  f.stages.push_back({"lane_infer", 3.2, 312.5});
  f.usage.cpu_percent = 40; f.usage.gpu_percent = std::nullopt;
  return f;
}

TEST(telemetry_has_fields_and_decimates) {
  const auto j = nlohmann::json::parse(telemetry_json(sample()));
  for (const char * k : {"t", "state", "mode", "reason", "readiness", "inspection", "vehicle", "lane", "plan", "detections", "memory", "stages", "usage", "params_version"})
    CHECK(j.contains(k));
  CHECK(j["reason"] == "say \"hi\"\n");
  CHECK(j["lane"]["left"].size() <= 40);
  CHECK(j["lane"]["left"].back()[0].get<double>() > 2.9);   // 끝점 유지
  CHECK(j["usage"]["gpu"].is_null());
  CHECK(j["readiness"]["lane_fresh"] == true);
  CHECK(j["inspection"]["models"][0]["status"] == "READY");
  CHECK(j["detections"][0]["cls"] == "con");
}

TEST(status_only_json_is_small) {
  auto f = sample();
  f.full = false;
  const auto s = telemetry_json(f);
  const auto j = nlohmann::json::parse(s);
  CHECK(j.contains("state") && j.contains("readiness") && j.contains("inspection"));
  CHECK(!j.contains("vehicle") && !j.contains("lane") && !j.contains("stages"));
  CHECK(s.size() < 600);
}
