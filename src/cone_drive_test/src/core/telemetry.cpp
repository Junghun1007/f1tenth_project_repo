#include "cone_drive_test/core/telemetry.hpp"
#include "cone_drive_test/core/json_writer.hpp"
#include <cmath>

namespace cone_drive_test {

namespace {
const char * detector_name(DetectorModel model) { return model == DetectorModel::Yolox ? "yolox" : "yolo11seg"; }

void points(JsonWriter & w, const char * key, const std::vector<Vec2> & pts, std::size_t max_points) {
  w.key(key).begin_array();
  const std::size_t n = pts.size();
  const std::size_t m = std::min(n, max_points);
  for (std::size_t k = 0; k < m; ++k) {
    const std::size_t i = m <= 1 ? 0 : (k * (n - 1)) / (m - 1);   // 처음·끝 점 유지
    w.begin_array().value(pts[i].x, 3).value(pts[i].y, 3).end_array();
  }
  w.end_array();
}
}  // namespace

std::string telemetry_json(const TelemetryFrame & f, std::size_t max_points) {
  JsonWriter w;
  w.begin_object();
  w.key("t").value(f.t, 3).key("state").value(f.state).key("mode").value(f.mode).key("reason").value(f.reason);
  const auto & r = f.readiness;
  w.key("readiness").begin_object()
   .key("attitude_ok").value(r.attitude_ok).key("lane_model_ready").value(r.lane_model_ready)
   .key("yolo_model_ready").value(r.yolo_model_ready).key("vesc_connected").value(r.vesc_connected)
   .key("erpm_fresh").value(r.erpm_fresh).key("lane_fresh").value(r.lane_fresh).key("client_alive").value(r.client_alive)
   .end_object();
  const auto & a = f.inspection;
  w.key("inspection").begin_object().key("attitude").begin_object()
   .key("status").value(a.attitude_status).key("height_m").value(a.height_m).key("roll_deg").value(a.roll_deg, 2)
   .key("pitch_down_deg").value(a.pitch_down_deg, 2).key("message").value(a.attitude_message).end_object()
   .key("models").begin_array();
  for (const auto & m : a.models)
    w.begin_object().key("name").value(m.name).key("status").value(m.status).key("stage").value(m.stage)
     .key("elapsed_s").value(m.elapsed_s, 1).key("message").value(m.message).end_object();
  w.end_array().end_object();
  w.key("stabilization").value(f.stabilization);
  w.key("detector").begin_object().key("active").value(detector_name(f.detector)).key("requested");
  if (f.requested_detector) w.value(detector_name(*f.requested_detector)); else w.null();
  w.key("can_switch").value(f.detector_switch_allowed).key("stage").value(f.detector_stage);
  if (f.full) {
    w.key("performance").begin_object();
    for (std::size_t i = 0; i < f.detector_perf.size(); ++i) {
      w.key(detector_name(static_cast<DetectorModel>(i)));
      if (!f.detector_perf[i]) { w.null(); continue; }
      const auto & p = *f.detector_perf[i];
      const auto & latest = p.latest;
      w.begin_object().key("detect_p95").value(p.detect_p95, 2).key("mask_p95").value(p.mask_p95, 2)
        .key("distance_p95").value(p.distance_p95, 2).key("total_p95").value(p.total_p95, 2).key("fps").value(p.fps, 1)
        .key("candidates").value((long long)latest.candidates).key("depth_reads").value((long long)latest.depth_reads)
        .key("no_depth").value((long long)latest.no_depth).key("insufficient").value((long long)latest.insufficient)
        .key("budget_exhausted").value((long long)latest.budget_exhausted)
        .key("ground_rejected").value((long long)latest.ground_rejected).end_object();
    }
    w.end_object();
  }
  w.end_object();
  if (f.full) {
    const auto & v = f.vehicle;
    w.key("vehicle").begin_object().key("speed").value(v.speed, 3).key("target").value(v.target, 3)
     .key("steer_deg").value(v.steer_deg, 2).key("cte").value(v.cte, 4).key("duty").value(v.duty, 4).key("brake").value(v.brake, 2)
     .end_object();
    w.key("lane").begin_object();
    points(w, "left", f.lane_left, max_points);
    points(w, "right", f.lane_right, max_points);
    points(w, "center", f.lane_center, max_points);
    points(w, "path", f.lane_path, max_points);
    w.end_object();
    w.key("plan").begin_object();
    points(w, "path", f.plan_path, max_points);
    w.key("mode").value(f.plan_mode).key("rollout_ok").value(f.rollout_ok).end_object();
    w.key("detections").begin_array();
    for (const auto & d : f.detections)
      w.begin_object().key("cls").value(d.cls).key("score").value(double(d.score), 2).key("x").value(d.x, 3).key("y").value(d.y, 3)
       .key("length").value(d.length, 3).key("width").value(d.width, 3).key("yaw").value(d.yaw, 3)
       .key("distance").value(d.distance, 3).end_object();
    w.end_array();
    w.key("memory_mode").value((long long)f.memory_mode);
    w.key("lock_m").value(f.lock_m);
    w.key("lon").begin_object().key("mode").value((long long)f.lon_mode).key("requested").value((long long)f.lon_mode_requested)
      .key("phase").value(f.lon_phase).end_object();
    w.key("traffic").begin_object().key("enabled").value(f.traffic_enabled).key("signal").value((long long)f.traffic.signal)
      .key("line").value(std::isfinite(f.traffic.line_m) ? f.traffic.line_m : -99.0).key("committed").value(f.traffic.committed)
      .key("holding").value(f.traffic.holding).key("cap").value(std::isfinite(f.traffic.cap) ? f.traffic.cap : -1.0).end_object();
    w.key("lead").begin_object();
    w.key("active").value(f.lead.active);
    w.key("seen").value(f.lead.seen);
    w.key("holding").value(f.lead.holding);
    w.key("gap").value(f.lead.gap);
    w.key("speed").value(f.lead.lead_speed);
    w.key("cap").value(std::isfinite(f.lead.cap) ? f.lead.cap : -1.0);
    w.end_object();
    w.key("memory").begin_array();
    for (const auto & m : f.memory)
      w.begin_object().key("x").value(m.x, 3).key("y").value(m.y, 3).key("confirmed").value(m.confirmed).key("id").value((long long)m.id).end_object();
    w.end_array();
    w.key("stages").begin_array();
    for (const auto & s : f.stages) w.begin_object().key("name").value(s.stage).key("ms").value(s.avg_ms, 2).key("fps").value(s.fps, 1).end_object();
    w.end_array();
    w.key("usage").begin_object().key("cpu").value(f.usage.cpu_percent, 1).key("proc_cpu").value(f.usage.process_cpu_percent, 1)
     .key("mem").value(f.usage.mem_percent, 1).key("gpu");
    if (f.usage.gpu_percent) w.value(*f.usage.gpu_percent, 1); else w.null();
    w.end_object();
    w.key("params_version").value((long long)f.params_version);
  }
  w.end_object();
  return w.str();
}

}  // namespace cone_drive_test
