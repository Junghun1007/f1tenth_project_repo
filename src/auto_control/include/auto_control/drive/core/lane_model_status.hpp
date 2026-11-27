// 차선 검출기(RGB line_detactor · IR ir_line_detactor)가 지금 실제로 쓰는 모델·정밀도.
// 검출기의 ~/model_status 문자열(bev_handoff::ModelLoadStatus)을 읽어 웹 "차선 모델" 패널에 보낸다.
//   "LOADING | model=<경로>" → "LOADING | <단계>" → "READY | model=<경로> | engine=<캐시> | precision=<p> | trial inference=PASS"
//   실행 중 교체: "READY | model=<지금 경로> | precision=<p> | swap=LOADING <파일> (<p>) | <단계>" / "... | swap=FAILED <이유>"
//   교체 끝: "READY | model=<새 경로> | ... | swapped"   (뒤에 " | elapsed=..s | stage_age=..s"가 붙는다)
#pragma once
#include <string>
#include <vector>

#include "auto_control/drive/core/json_writer.hpp"

namespace auto_control::drive {

struct LaneModelLive {
  std::string state{"WAITING"};   // WAITING / LOADING / READY / FAILED
  std::string path;               // 지금 추론에 쓰는 ONNX (READY 전에는 불러오는 중인 파일)
  std::string precision;          // fp32 / fp16 / int8 (상태 문자열에 없으면 LaneResult.engine_precision)
  std::string swap;               // "" / LOADING / FAILED  (실행 중 교체)
  std::string swap_target;        // 교체할 파일 (정밀도 포함)
  std::string detail;             // 불러오는 단계 또는 실패 이유
};

inline std::vector<std::string> split_model_status(const std::string & s) {
  std::vector<std::string> out;
  std::size_t from = 0;
  while (true) {
    const auto bar = s.find(" | ", from);
    out.push_back(s.substr(from, bar == std::string::npos ? std::string::npos : bar - from));
    if (bar == std::string::npos) break;
    from = bar + 3;
  }
  return out;
}

inline void apply_model_status(LaneModelLive & m, const std::string & s) {
  const auto parts = split_model_status(s);
  const std::string & head = parts.front();
  if (head != "READY" && head != "LOADING" && head != "FAILED") return;
  auto starts = [](const std::string & v, const char * p) { return v.rfind(p, 0) == 0; };
  auto meta = [&](const std::string & v) {   // ModelLoadStatus가 붙이는 항목
    return starts(v, "elapsed=") || starts(v, "stage_age=") || starts(v, "monitor_seq=");
  };
  m.state = head;
  m.swap.clear(); m.swap_target.clear(); m.detail.clear();
  bool after_swap_loading = false;
  for (std::size_t i = 1; i < parts.size(); ++i) {
    const auto & p = parts[i];
    if (meta(p)) continue;
    if (starts(p, "model=")) { m.path = p.substr(6); continue; }
    if (starts(p, "precision=")) { m.precision = p.substr(10); continue; }
    if (starts(p, "engine=") || p == "swapped" || starts(p, "trial inference=")) continue;
    if (starts(p, "swap=LOADING ")) { m.swap = "LOADING"; m.swap_target = p.substr(13); after_swap_loading = true; continue; }
    if (starts(p, "swap=FAILED")) { m.swap = "FAILED"; m.detail = p.size() > 12 ? p.substr(12) : ""; continue; }
    if (head == "READY" && !after_swap_loading) continue;
    if (m.detail.empty()) m.detail = p;   // LOADING 단계 · 교체 단계 · 실패 이유
  }
}

// 웹에 보이는 짧은 이름: 번호표(launch_support.py LANE_MODELS)의 파일 이름으로 정한다
inline std::string lane_model_label(const std::string & path) {
  const auto slash = path.find_last_of('/');
  const std::string file = slash == std::string::npos ? path : path.substr(slash + 1);
  if (file.empty()) return "";
  if (file.find("lane_c3") != std::string::npos) return "C3";
  if (file.find("stop_line_v5") != std::string::npos) return "v5";
  if (file.find("ir_stereo_bev") != std::string::npos) return "IR 기존";
  if (file.find("fast_scnn_stop_line_120x300") != std::string::npos) return "기존 stop_0910";
  return file;
}

inline std::string lane_model_file(const std::string & path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

// {"state","model","file","precision","swap","swap_target","detail","want_model","want_precision"}
inline void write_lane_model_json(JsonWriter & w, const LaneModelLive & m, const std::string & want_path,
                                  const std::string & want_precision) {
  w.begin_object();
  w.key("state").value(m.state);
  w.key("model").value(lane_model_label(m.path));
  w.key("file").value(lane_model_file(m.path));
  w.key("precision").value(m.precision);
  w.key("swap").value(m.swap);
  w.key("swap_target").value(m.swap_target);
  w.key("detail").value(m.detail);
  // 웹에서 고른 모델 (빈 경로 = 그 검출기의 기본 모델 그대로)
  w.key("want_model").value(want_path.empty() ? std::string() : lane_model_label(want_path));
  w.key("want_precision").value(want_precision);
  w.end_object();
}

}  // namespace auto_control::drive
