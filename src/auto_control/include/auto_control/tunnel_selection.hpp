#pragma once
// C++ port of auto_control_tunnel/{path_validation,selection,telemetry}.py.
// Templated on the lane message so the policy has no ROS dependency (LaneResult in production).
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace auto_control::tunnel
{

template<typename Msg>
std::int64_t stamp_ns(const Msg & message)
{
  return static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL + message.header.stamp.nanosec;
}

inline bool close_rel(double a, double b, double rel = 1e-4)
{
  return std::abs(a - b) <= rel * std::max(std::abs(a), std::abs(b));
}

struct PathRules
{
  std::string frame_id{"front_axle_bev"};
  int minimum_points{8};
  double minimum_span{.12}, minimum_x{.05}, maximum_x{2.20}, maximum_gap{.15};
  double x_max{3.}, y_max{.6}, mpp{.01};

  void validate() const
  {
    const auto positive = [](double v) {return std::isfinite(v) && v > 0;};
    if (frame_id.empty() || minimum_points < 3 || !positive(minimum_span) || !positive(maximum_x) ||
      !positive(maximum_gap) || !positive(x_max) || !positive(y_max) || !positive(mpp) ||
      !std::isfinite(minimum_x) || !(0 <= minimum_x && minimum_x < maximum_x && maximum_x <= x_max))
    {
      throw std::invalid_argument("Invalid tunnel path geometry rules");
    }
  }
};

// Controller-compatible readiness: canvas, support and contiguous-path requirements.
template<typename Msg>
bool metric_path_ready(const Msg & m, const PathRules & rules)
{
  const double width = m.source_width, height = m.source_height;
  if (width <= 0 || height <= 0) {return false;}
  const double sx = m.centerline_bev_width_m / width, sy = m.centerline_bev_height_m / height;
  const std::size_t count = m.centerline_points.size();
  if (!m.centerline_valid || m.centerline_sample_limit_reached || m.state < 1 || m.state > 3 ||
    m.header.frame_id != rules.frame_id || !std::isfinite(sx) || !std::isfinite(sy) || sx <= 0 || sy <= 0 ||
    !close_rel(sx, rules.mpp) || !close_rel(sy, rules.mpp) || !close_rel(width * sx, 2 * rules.y_max) ||
    !close_rel(height * sy, rules.x_max) || count < 2 || count > 10000 || count != m.centerline_support.size())
  {
    return false;
  }
  std::vector<std::pair<double, double>> raw;
  raw.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & p = m.centerline_points[i];
    const auto support = m.centerline_support[i];
    if (support < 1 || support > 4 || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      std::abs(p.z) > 1e-6 || p.x < 0 || p.x >= width + m.padding_left + m.padding_right ||
      p.y < 0 || p.y >= height)
    {
      return false;
    }
    raw.emplace_back(rules.x_max - (p.y + .5) * sy, rules.y_max - (p.x - m.padding_left + .5) * sx);
  }
  std::vector<std::pair<double, double>> points;
  double span = 0.;
  for (const auto & p : raw) {
    if (!(rules.minimum_x <= p.first && p.first <= rules.maximum_x)) {
      if (!points.empty()) {break;}
      continue;
    }
    if (!points.empty()) {
      const double gap = std::hypot(p.first - points.back().first, p.second - points.back().second);
      if (gap < 1e-5) {continue;}
      if (gap > rules.maximum_gap) {break;}
      span += gap;
    }
    points.push_back(p);
  }
  return static_cast<int>(points.size()) >= rules.minimum_points && span >= rules.minimum_span;
}

// 차선 입력 선택: TCN 판정(또는 강제 IR)이 오면 그 자리에서 바로 전환한다 (기본).
// 실험 기능 두 가지는 실행 인자로만 켠다 (기본 꺼짐 → 기존 동작과 같음):
//  - 사전 준비(prearm): TCN 점수가 prearm_score를 넘으면 선택은 그대로 두고 상태만 ENTERING/EXITING으로 바꿔
//    카메라·IR BEV·IR 추론을 미리 켠다 (camera()/compute()). 점수가 prearm_hold 동안 기준 아래면 취소.
//  - 유효 경로 전환(valid handover): TCN이 전환을 요청해도 새 출처가 유효한 경로(metric_path_ready)를 낼 때까지
//    지금 출처가 유효한 동안 최대 valid_handover_max 만큼 지금 출처를 계속 넘긴다.
// 그 밖에는 같은 출처에서 시각이 되돌아간(순서가 뒤바뀐) 결과를 버리는 것뿐이다.
struct TransitionOptions
{
  bool prearm_entry{false}, prearm_exit{false};
  double prearm_score{.3}, prearm_hold_sec{1.};
  bool valid_handover{false};
  double valid_handover_max_sec{.5};

  void validate() const
  {
    if (!std::isfinite(prearm_score) || !(0 < prearm_score && prearm_score <= 1) ||
      !std::isfinite(prearm_hold_sec) || prearm_hold_sec <= 0 ||
      !std::isfinite(valid_handover_max_sec) || valid_handover_max_sec <= 0)
    {
      throw std::invalid_argument("Invalid tunnel transition options");
    }
  }
};

class LaneSelector
{
public:
  LaneSelector(double max_age, bool validation_mode, PathRules rules, TransitionOptions options = {})
  : validation_mode(validation_mode), max_age(max_age), rules(std::move(rules)), options(options)
  {
    if (!std::isfinite(max_age) || max_age <= 0) {throw std::invalid_argument("Invalid lane freshness setting");}
    this->rules.validate();
    this->options.validate();
  }

  // 요청한 출처의 유효 경로를 기다리는 중
  bool transitioning() const {return requested != selected;}
  bool prearmed() const {return prearm_since_ns.has_value();}
  // 카메라(IR 노출·게이트)는 터널 밖(OUTSIDE)만 RGB, 계산은 전환·준비 중이면 둘 다
  std::string compute() const {return state == "OUTSIDE" ? "RGB" : state == "INSIDE" ? "IR" : "BOTH";}
  std::string camera() const {return state == "OUTSIDE" ? "RGB" : "IR";}

  // TCN 판정: entry → IR, exit → RGB. 이전 보고보다 시각이 늦기만 하면 따른다
  bool event(const std::string & kind, std::int64_t stamp, std::int64_t now_ns)
  {
    if ((kind != "entry" && kind != "exit") || stamp <= last_event_ns) {return false;}
    last_event_ns = stamp;
    const bool entry = kind == "entry";
    return request_source(entry ? "IR" : "RGB", now_ns, entry ? "tcn_entry" : "tcn_exit", options.valid_handover);
  }

  // 강제 IR 켬/끔은 항상 바로 전환. wait_valid면 새 출처의 유효 경로를 기다린다
  bool request_source(std::string target, std::int64_t now_ns,
    const char * immediate_reason = nullptr, bool wait_valid = false)
  {
    if (target == requested && target == selected && !prearmed()) {return false;}
    const std::string why = immediate_reason ? immediate_reason : (target == "IR" ? "force_ir_on" : "force_ir_off");
    requested = target;
    prearm_since_ns.reset();
    if (target == selected) {   // 기다리던 전환 취소 또는 준비 취소
      finish(now_ns, transition_since_ns ? "reversed_request" : why);
      return true;
    }
    if (wait_valid && current_valid) {
      transition_since_ns = now_ns;
      update_state(std::string("waiting_valid_") + (target == "IR" ? "ir" : "rgb"));
      return true;
    }
    transition_since_ns = now_ns;
    finish(now_ns, why);
    return true;
  }

  // TCN 점수(사전 준비). 상태가 바뀌면 true
  bool scores(std::optional<double> entry, std::optional<double> exit, bool live, std::int64_t now_ns)
  {
    if (transitioning()) {return false;}
    const bool inside = selected == "IR";
    if (!(inside ? options.prearm_exit : options.prearm_entry)) {return false;}
    const auto score = inside ? exit : entry;
    if (live && score && *score >= options.prearm_score) {
      prearm_high_ns = now_ns;
      if (prearmed()) {return false;}
      prearm_since_ns = now_ns;
      update_state(inside ? "prearm_exit" : "prearm_entry");
      return true;
    }
    return tick(now_ns);
  }

  // 시간 경과: 유효 경로 대기 상한·사전 준비 취소. 상태가 바뀌면 true
  bool tick(std::int64_t now_ns)
  {
    if (transitioning() && now_ns - *transition_since_ns >= seconds_ns(options.valid_handover_max_sec)) {
      finish(now_ns, "handover_timeout");
      return true;
    }
    if (prearmed() && now_ns - prearm_high_ns >= seconds_ns(options.prearm_hold_sec)) {
      prearm_since_ns.reset();
      update_state("prearm_cancelled");
      return true;
    }
    return false;
  }

  // True when this message is the selected source and should be forwarded.
  template<typename Msg>
  bool lane(
    const std::string & source, const Msg & m, std::int64_t now_ns)
  {
    auto last = last_input_ns.find(source);
    if (last == last_input_ns.end()) {return false;}
    const auto stamp = stamp_ns(m);
    if (stamp <= last->second) {return false;}
    last->second = stamp;
    const bool ready = metric_path_ready(m, rules);
    if (transitioning()) {
      if (source == requested && ready) {finish(now_ns, "first_valid_target");}
      else if (source == selected && !ready) {finish(now_ns, "current_invalid"); return false;}
      else {tick(now_ns);}
    }
    if (source == selected) {current_valid = ready;}   // 전환을 끝낸 결과도 포함
    return source == selected;
  }

  bool validation_mode;
  double max_age;
  PathRules rules;
  TransitionOptions options;
  std::string state{"OUTSIDE"}, selected{"RGB"}, requested{"RGB"}, reason{"steady"};
  std::string last_handover;  // JSON text for the log (선택기가 기록한 뒤 비움)
  std::int64_t last_event_ns{0};
  std::map<std::string, std::int64_t> last_input_ns{{"RGB", 0}, {"IR", 0}};

private:
  static std::int64_t seconds_ns(double s) {return static_cast<std::int64_t>(s * 1e9);}

  void update_state(const std::string & why)
  {
    reason = why;
    if (transitioning()) {state = requested == "IR" ? "ENTERING" : "EXITING";}
    else if (prearmed()) {state = selected == "RGB" ? "ENTERING" : "EXITING";}
    else {state = selected == "IR" ? "INSIDE" : "OUTSIDE";}
  }

  void finish(std::int64_t now_ns, const std::string & why)
  {
    const double total_ms = transition_since_ns ? (now_ns - *transition_since_ns) / 1e6 : 0.;
    if (selected != requested) {current_valid = false;}   // 새 출처의 유효 여부는 다음 결과에서
    std::ostringstream out;
    out << "{\"target\": \"" << requested << "\", \"reason\": \"" << why << "\", \"total_ms\": " << total_ms << "}";
    last_handover = out.str();
    selected = requested;
    transition_since_ns.reset();
    prearm_since_ns.reset();
    update_state(why);
  }

  std::optional<std::int64_t> transition_since_ns, prearm_since_ns;
  std::int64_t prearm_high_ns{0};
  bool current_valid{false};   // 지금 넘기는 출처의 마지막 결과가 유효 경로였는지
};

// ---- Minimal JSON reader for the TCN report (flat object with one-level nested objects) ----
// Returns raw value text per top-level key: strings keep quotes, objects keep braces.
inline std::map<std::string, std::string> json_object(const std::string & text)
{
  std::map<std::string, std::string> out;
  std::size_t i = text.find('{');
  if (i == std::string::npos) {throw std::invalid_argument("not an object");}
  ++i;
  const auto skip = [&]() {while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {++i;}};
  const auto read_string = [&]() {
      if (text[i] != '"') {throw std::invalid_argument("expected string");}
      const std::size_t start = i++;
      while (i < text.size() && text[i] != '"') {i += text[i] == '\\' ? 2 : 1;}
      if (i >= text.size()) {throw std::invalid_argument("unterminated string");}
      return text.substr(start, ++i - start);
    };
  while (true) {
    skip();
    if (i >= text.size()) {throw std::invalid_argument("unterminated object");}
    if (text[i] == '}') {break;}
    if (text[i] == ',') {++i; continue;}
    const std::string key = read_string();
    skip();
    if (i >= text.size() || text[i] != ':') {throw std::invalid_argument("expected colon");}
    ++i;
    skip();
    const std::size_t start = i;
    if (text[i] == '"') {
      read_string();
    } else if (text[i] == '{' || text[i] == '[') {
      int depth = 0;
      for (; i < text.size(); ++i) {
        if (text[i] == '"') {read_string(); --i; continue;}
        if (text[i] == '{' || text[i] == '[') {++depth;}
        if (text[i] == '}' || text[i] == ']') {if (--depth == 0) {++i; break;}}
      }
    } else {
      while (i < text.size() && text[i] != ',' && text[i] != '}') {++i;}
    }
    std::string value = text.substr(start, i - start);
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {value.pop_back();}
    out[key.substr(1, key.size() - 2)] = value;
  }
  return out;
}

inline std::optional<std::string> json_string(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.size() < 2 || found->second.front() != '"') {return std::nullopt;}
  return found->second.substr(1, found->second.size() - 2);
}

inline std::optional<std::int64_t> json_int(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.empty() ||
    found->second.find_first_not_of("-0123456789") != std::string::npos)
  {
    return std::nullopt;
  }
  return std::stoll(found->second);
}

inline std::optional<double> json_double(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.empty() || found->second == "null" || found->second.front() == '"' ||
    found->second.front() == '{' || found->second == "true" || found->second == "false")
  {
    return std::nullopt;
  }
  const double value = std::stod(found->second);
  return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
}

// Port of TunnelPreviewTelemetry: three HUD lines for /tunnel/preview_status.
class PreviewTelemetry
{
public:
  void update(const std::map<std::string, std::string> & report, double now)
  {
    const auto stamp = json_int(report, "source_stamp_ns");
    if (!stamp) {return;}
    const std::string status = json_string(report, "status").value_or("WAITING");
    const bool reset = (last_stamp_ && *stamp < *last_stamp_) ||
      (status.rfind("LIVE", 0) == 0 && status_.rfind("LIVE", 0) != 0);
    if (reset) {infer_sum_ms_ = 0.; infer_count_ = 0;}
    const bool duplicate = last_stamp_ && *stamp == *last_stamp_;
    last_stamp_ = stamp;
    received_at_ = now;
    status_ = status;
    const auto scores = report.find("scores");
    if (scores != report.end() && !scores->second.empty() && scores->second.front() == '{') {
      const auto values = json_object(scores->second);
      entry_ = json_double(values, "entry");
      exit_ = json_double(values, "exit");
    }
    if (status.rfind("LIVE", 0) != 0 || duplicate) {return;}
    const auto ms = json_double(report, "inference_wall_ms");
    if (ms && *ms > 0) {infer_sum_ms_ += *ms; ++infer_count_;}
  }

  std::string text(const std::string & state, double now) const
  {
    const bool fresh = received_at_ && 0 <= now - *received_at_ && now - *received_at_ <= .5;
    std::string status = !received_at_ ? "WAITING" : !fresh ? "STALE" : status_.substr(0, status_.find(' '));
    const auto fixed = [](double v, int digits) {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(digits);
        out << v;
        return out.str();
      };
    std::string fps = "--";
    if (fresh && status_.rfind("LIVE", 0) == 0 && infer_count_ && infer_sum_ms_ > 0) {
      fps = fixed(1000.0 * infer_count_ / infer_sum_ms_, 1);
    }
    const std::string entry = fresh && entry_ ? fixed(*entry_, 2) : "--";
    const std::string exit = fresh && exit_ ? fixed(*exit_, 2) : "--";
    return "Tunnel: " + state + "\nTCN avg " + fps + "FPS\nTCN " + status + " E:" + entry + " X:" + exit;
  }

private:
  std::optional<double> received_at_;
  std::optional<std::int64_t> last_stamp_;
  std::string status_{"WAITING"};
  std::optional<double> entry_, exit_;
  double infer_sum_ms_{0.};
  int infer_count_{0};
};

}  // namespace auto_control::tunnel
