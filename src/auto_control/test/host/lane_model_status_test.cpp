#include "check.hpp"
#include "auto_control/drive/core/lane_model_status.hpp"
#include <nlohmann/json.hpp>

using namespace auto_control::drive;

TEST(lane_model_status_follows_load_ready_swap) {
  LaneModelLive m;
  CHECK(m.state == "WAITING");
  apply_model_status(m, "LOADING | model=/s/models/lane_c3_120x300.int8.qdq.onnx | elapsed=0s | stage_age=0s | monitor_seq=1");
  CHECK(m.state == "LOADING");
  CHECK(lane_model_label(m.path) == "C3");
  apply_model_status(m, "LOADING | TensorRT engine build | elapsed=40s | stage_age=38s | monitor_seq=40");
  CHECK(m.state == "LOADING");
  CHECK(m.detail == "TensorRT engine build");
  CHECK(lane_model_label(m.path) == "C3");   // 단계 문자열에는 경로가 없으므로 앞 값 유지

  apply_model_status(m, "READY | model=/s/models/lane_c3_120x300.int8.qdq.onnx | engine=/c/x.engine | precision=int8 | "
                        "trial inference=PASS | elapsed=60s | stage_age=0s");
  CHECK(m.state == "READY");
  CHECK(m.precision == "int8");
  CHECK(m.swap.empty());
  CHECK(m.detail.empty());

  apply_model_status(m, "READY | model=/s/models/lane_c3_120x300.int8.qdq.onnx | precision=int8 | swap=LOADING lane_c3_120x300.onnx (fp32)"
                        " | Building engine | elapsed=70s | stage_age=3s");
  CHECK(m.state == "READY");
  CHECK(m.swap == "LOADING");
  CHECK(m.swap_target == "lane_c3_120x300.onnx (fp32)");
  CHECK(m.detail == "Building engine");
  CHECK(m.precision == "int8");   // 교체 끝나기 전에는 지금 모델 그대로

  apply_model_status(m, "READY | model=/s/models/lane_c3_120x300.int8.qdq.onnx | precision=int8 | swap=FAILED no file | elapsed=71s");
  CHECK(m.swap == "FAILED");
  CHECK(m.detail == "no file");

  apply_model_status(m, "READY | model=/s/models/lane_c3_120x300.onnx | engine=/c/y.engine | precision=fp32 | trial inference=PASS | swapped");
  CHECK(m.swap.empty());
  CHECK(m.precision == "fp32");
  CHECK(lane_model_file(m.path) == "lane_c3_120x300.onnx");

  apply_model_status(m, "FAILED | Three-channel ONNX model not found: x | elapsed=1s");
  CHECK(m.state == "FAILED");
  CHECK(m.detail == "Three-channel ONNX model not found: x");
  apply_model_status(m, "garbage");
  CHECK(m.state == "FAILED");
}

TEST(lane_model_labels_and_json) {
  CHECK(lane_model_label("/a/fast_scnn_stop_line_v5_120x300_batch_1.int8.qdq.onnx") == "v5");
  CHECK(lane_model_label("/a/fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx") == "기존 stop_0910");
  CHECK(lane_model_label("/a/ir_stereo_bev_120x300.onnx") == "IR 기존");
  CHECK(lane_model_label("/a/other.onnx") == "other.onnx");
  CHECK(lane_model_label("").empty());
  LaneModelLive m;
  apply_model_status(m, "READY | model=/a/ir_stereo_bev_120x300.onnx | engine=e | precision=fp32 | trial inference=PASS");
  JsonWriter w;
  write_lane_model_json(w, m, "/a/lane_c3_120x300.onnx", "fp32");
  const auto j = nlohmann::json::parse(w.str());
  CHECK(j["state"] == "READY");
  CHECK(j["model"] == "IR 기존");
  CHECK(j["precision"] == "fp32");
  CHECK(j["want_model"] == "C3");
  CHECK(j["want_precision"] == "fp32");
  JsonWriter w2;
  write_lane_model_json(w2, m, "", "");
  CHECK(nlohmann::json::parse(w2.str())["want_model"] == "");
}
