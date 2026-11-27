#include "check.hpp"
#include <cmath>
#include "auto_control/drive/core/telemetry.hpp"
#include <cmath>
#include <nlohmann/json.hpp>

using namespace auto_control::drive;

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
  for (const char * k : {"t", "state", "mode", "reason", "readiness", "inspection", "vehicle", "lane", "plan", "detections", "memory", "stages", "usage"})
    CHECK(j.contains(k));
  CHECK(!j.contains("params_version") && !j.contains("stabilization"));   // 화면이 쓰지 않는 항목은 보내지 않음
  CHECK(!j["lane"].contains("center"));
  CHECK(!j["lane"].contains("age_s"));   // 차선 없음(-1)이면 생략
  CHECK(j["memory"][0].is_array() && j["memory"][0].size() == 2);
  CHECK(j["reason"] == "say \"hi\"\n");
  CHECK(j["lane"]["left"].size() <= 40);
  CHECK(j["lane"]["left"].back()[0].get<double>() > 2.9);   // 끝점 유지
  CHECK(j["usage"]["gpu"].is_null());
  CHECK(j["readiness"]["lane_fresh"] == true);
  CHECK(j["inspection"]["models"][0]["status"] == "READY");
  CHECK(j["detections"][0]["cls"] == "con");
}

TEST(telemetry_reports_lane_age) {
  auto f = sample();
  f.lane_age_s = 1.234;
  const auto j = nlohmann::json::parse(telemetry_json(f));
  CHECK(std::abs(j["lane"]["age_s"].get<double>() - 1.23) < 1e-9);
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

TEST(tunnel_tcn_preserves_events_across_status_updates_and_expires_old_scores) {
  TunnelTcnTelemetry tcn;
  CHECK(nlohmann::json::parse(tcn.json(1.0))["status"] == "WAITING");
  tcn.update(R"({"scores":{"entry":0.91,"exit":0.05},"state":"inside","status":"LIVE","threshold":0.55,"event":{"type":"entry"}})", 1.0);
  tcn.update(R"({"scores":{"entry":0.10,"exit":0.07},"state":"inside","status":"LIVE","threshold":0.55,"event":null})", 1.2);
  tcn.update(R"({"scores":{"entry":null,"exit":0.05},"status":"LIVE"})", 1.3);

  const auto live = nlohmann::json::parse(tcn.json(1.5));
  CHECK(live["status"] == "LIVE");
  CHECK(live["state"] == "inside");
  CHECK(live["threshold"] == 0.55);
  CHECK(live["samples"].size() == 2);
  CHECK(live["samples"][0][3] == "entry");
  CHECK(live["samples"][1].size() == 3);
  const auto stale = nlohmann::json::parse(tcn.json(3.0));
  CHECK(stale["status"] == "STALE");
  CHECK(stale["samples"].empty());
}

TEST(tunnel_tcn_marks_manual_transitions) {
  TunnelTcnTelemetry tcn;
  tcn.update(R"({"scores":{"entry":0.07,"exit":0.05},"state":"outside","status":"LIVE","state_override":"outside","event":{"type":"exit","forced":true}})", 1.0);
  const auto j = nlohmann::json::parse(tcn.json(1.1));
  CHECK(j["state"] == "outside");
  CHECK(j["state_override"] == "outside");
  CHECK(j["samples"][0][3] == "exit");
  CHECK(j["samples"][0][4] == true);
}

TEST(tunnel_tcn_reports_inference_timing) {   // 웹 단계별 연산의 TCN 행 (추론·처리 ms, 입력 FPS, 입력 지연)
  TunnelTcnTelemetry tcn;
  const auto t = tcn.update(R"({"scores":{"entry":0.1,"exit":0.2},"state":"outside","status":"LIVE",)"
                            R"("inference_wall_ms":1.9,"processing_ms":3.4,"source_fps":29.8,"source_age_at_receive_ms":21.0})", 1.0);
  CHECK(t.infer_ms && std::abs(*t.infer_ms - 1.9) < 1e-9);
  CHECK(t.processing_ms && std::abs(*t.processing_ms - 3.4) < 1e-9);
  CHECK(t.source_fps && std::abs(*t.source_fps - 29.8) < 1e-9);
  CHECK(t.source_age_ms && std::abs(*t.source_age_ms - 21.0) < 1e-9);
  const auto waiting = tcn.update(R"({"scores":{"entry":null,"exit":null},"status":"WARMUP","processing_ms":0.4})", 1.1);
  CHECK(!waiting.infer_ms && waiting.processing_ms);   // 점수가 없어도 있는 값은 읽는다
  CHECK(!tcn.update("not json", 1.2).processing_ms);
}

TEST(telemetry_reports_imu_calibration_only_when_used) {
  auto f = sample();
  CHECK(!nlohmann::json::parse(telemetry_json(f))["readiness"].contains("imu_calibrated"));
  f.readiness.imu_used = true;
  f.readiness.imu_calibrated = false;
  CHECK(nlohmann::json::parse(telemetry_json(f))["readiness"]["imu_calibrated"] == false);
}

TEST(inspection_shows_imu_calibration_row_only_when_used) {
  auto f = sample();
  CHECK(!nlohmann::json::parse(telemetry_json(f))["inspection"].contains("imu"));
  f.inspection.imu_status = "CALIBRATING";
  f.inspection.imu_message = "보정 2.1/4.0 s · 차를 움직이지 마세요";
  f.full = false;   // 프리뷰 꺼짐(상태만)에서도 점검표는 보냄
  const auto j = nlohmann::json::parse(telemetry_json(f));
  CHECK(j["inspection"]["imu"]["status"] == "CALIBRATING");
  CHECK(j["inspection"]["imu"]["message"] == "보정 2.1/4.0 s · 차를 움직이지 마세요");
  CHECK(j["inspection"]["models"].size() == 1);
}
