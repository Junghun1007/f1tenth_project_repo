// 녹화한 BEV로 차선 검출 → 경로 생성을 다시 돌려 보는 오프라인 도구.
// line_detactor의 lane_connector.cpp·centerline.cpp를 그대로 링크하고, 파라미터 적용은
// line_detactor_node.cpp frame_lane_settings()와 같은 방식(bev_handoff X-매크로)으로 한다.
//
//   lane_path_lab infer <model.onnx> <rgb_bev 폴더> <logit 캐시 폴더>
//       각 BEV(120×300)를 모델에 넣고 3채널 logit(float32, 3×300×120)을 <프레임>.bin으로 저장
//   lane_path_lab serve
//       stdin 한 줄 = "<logit.bin 경로> key=value key=value ..." → stdout 한 줄 JSON
#include "line_detactor/centerline.hpp"
#include "line_detactor/lane_connector.hpp"
#include "bev_handoff/lane_tuning.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace line_detactor;
namespace fs = std::filesystem;

namespace {
constexpr int kW = 120, kH = 300, kPadding = 30;   // line_detactor.yaml result_padding_px

int infer(const std::string & model, const std::string & in_dir, const std::string & out_dir) {
  auto net = cv::dnn::readNetFromONNX(model);
  fs::create_directories(out_dir);
  std::vector<fs::path> files;
  for (const auto & e : fs::directory_iterator(in_dir))
    if (e.path().extension() == ".jpg" || e.path().extension() == ".png") files.push_back(e.path());
  std::sort(files.begin(), files.end());
  for (const auto & f : files) {
    cv::Mat bgr = cv::imread(f.string(), cv::IMREAD_COLOR);
    if (bgr.cols != kW || bgr.rows != kH) {std::cerr << "skip size " << f << "\n"; continue;}
    // 차량과 같은 전처리: BGR→RGB, /255, NCHW
    cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(kW, kH), cv::Scalar(), true, false, CV_32F);
    net.setInput(blob);
    cv::Mat out = net.forward();
    if (out.total() != static_cast<size_t>(3 * kW * kH)) {std::cerr << "bad output\n"; return 1;}
    std::ofstream o(fs::path(out_dir) / (f.stem().string() + ".bin"), std::ios::binary);
    o.write(reinterpret_cast<const char *>(out.ptr<float>()), sizeof(float) * out.total());
    std::cout << f.filename().string() << "\n";
  }
  return 0;
}

std::string b64(const unsigned char * p, size_t n) {
  static const char * t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string s;
  s.reserve((n + 2) / 3 * 4);
  for (size_t i = 0; i < n; i += 3) {
    const unsigned v = p[i] << 16 | (i + 1 < n ? p[i + 1] << 8 : 0) | (i + 2 < n ? p[i + 2] : 0);
    s += t[v >> 18 & 63]; s += t[v >> 12 & 63];
    s += i + 1 < n ? t[v >> 6 & 63] : '='; s += i + 2 < n ? t[v & 63] : '=';
  }
  return s;
}

void put_points(std::ostream & o, const std::vector<cv::Point2f> & pts) {
  o << '[';
  for (size_t i = 0; i < pts.size(); ++i) o << (i ? "," : "") << '[' << pts[i].x << ',' << pts[i].y << ']';
  o << ']';
}

std::string run(const std::string & line) {
  std::istringstream in(line);
  std::string path, kv;
  in >> path;
  bev_handoff::LaneTuning tuning;
  while (in >> kv) {
    const auto eq = kv.find('=');
    if (eq == std::string::npos) continue;
    const std::string k = kv.substr(0, eq);
    const double v = std::stod(kv.substr(eq + 1));
#define SET(type, key, member, label, unit, lo, hi, rgb, ir) \
    if (k == #key) tuning.key = std::is_same_v<type, int> ? static_cast<type>(std::lround(v)) : static_cast<type>(v);
    BEV_LANE_DETECTION_SETTINGS(SET)
    BEV_LANE_CONNECTION_SETTINGS(SET)
    BEV_LANE_CENTERLINE_SETTINGS(SET)
    BEV_LANE_CORNER_SETTINGS(SET)
    BEV_LANE_SMOOTHING_SETTINGS(SET)
#undef SET
  }
  LaneConnectionConfig connection;
  connection.ccl_serial_enabled = true;
  connection.padding_px = kPadding;
  CenterlineConfig centerline;
  centerline.bev_width_m = 1.2;
  centerline.bev_height_m = 3.0;
#define APPLY_CONNECTION(type, key, member, label, unit, lo, hi, rgb, ir) connection.member = tuning.key;
  BEV_LANE_CONNECTION_SETTINGS(APPLY_CONNECTION)
#undef APPLY_CONNECTION
#define APPLY_CENTERLINE(type, key, member, label, unit, lo, hi, rgb, ir) centerline.member = tuning.key;
  BEV_LANE_CENTERLINE_SETTINGS(APPLY_CENTERLINE)
  BEV_LANE_CORNER_SETTINGS(APPLY_CENTERLINE)
  BEV_LANE_SMOOTHING_SETTINGS(APPLY_CENTERLINE)
#undef APPLY_CENTERLINE

  std::vector<float> logits(3 * kW * kH);
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.read(reinterpret_cast<char *>(logits.data()), sizeof(float) * logits.size()))
      return "{\"error\":\"logit 파일을 읽지 못함\"}";
  }
  const auto started = std::chrono::steady_clock::now();
  // cuda_lane_pipeline.cu lane_labels_kernel과 같은 규칙 (임계값은 logit으로 변환해 비교)
  const auto logit = [](double p) {return static_cast<float>(std::log(p / (1.0 - p)));};
  const float lt = logit(std::clamp(tuning.left_mask_threshold, 1e-6, 1 - 1e-6));
  const float rt = logit(std::clamp(tuning.right_mask_threshold, 1e-6, 1 - 1e-6));
  const float st = logit(std::clamp(tuning.stop_line_mask_threshold, 1e-6, 1 - 1e-6));
  cv::Mat labels(kH, kW, CV_8UC1), stop(kH, kW, CV_8UC1);
  const int n = kW * kH;
  for (int i = 0; i < n; ++i) {
    const float l = logits[i], r = logits[n + i];
    labels.data[i] = l >= lt && (!(r >= rt) || l >= r) ? 1 : (r >= rt ? 2 : 0);
    stop.data[i] = logits[2 * n + i] >= st ? 255 : 0;
  }
  std::ostringstream o;
  o.precision(5);
  try {
    validate_lane_connection(connection);
    validate_centerline(centerline);
    if (!connection.enabled) return "{\"error\":\"connection_enabled=0이면 차량에서도 경로를 만들지 않음\"}";
    auto result = connect_lane_fragments(labels, connection, true, nullptr, false);
    const bool use_bridges = tuning.bridge_centerline && centerline.enabled;
    auto completed = use_bridges ? lane_geometry_with_bridges(result.labels, connection, nullptr) : std::nullopt;
    const auto & geometry = completed ? *completed : result;
    CenterlineResult center;
    std::vector<cv::Point2f> reference;
    if (centerline.enabled) {
      center = generate_centerline(geometry.labels, geometry.observed_paths, kW, kPadding, centerline, false, nullptr);
      auto ref_cfg = centerline;
      ref_cfg.corner_outer_enabled = false;   // 코너 바깥 반영을 끈 기준선 (비교용)
      reference = generate_centerline(geometry.labels, geometry.observed_paths, kW, kPadding, ref_cfg, false, nullptr).points;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    cv::Mat stop_padded;
    cv::copyMakeBorder(stop, stop_padded, 0, 0, kPadding, kPadding, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::Mat combined = result.labels.clone();   // 0 배경, 1/2 좌/우 모델, 3/4 좌/우 보간, 5 정지선
    combined.setTo(5, stop_padded & (combined == 0));
    o << "{\"width\":" << combined.cols << ",\"height\":" << combined.rows << ",\"padding\":" << kPadding
      << ",\"state\":" << int(result.state) << ",\"ms\":" << ms << ",\"bridges_used\":" << (completed ? "true" : "false")
      << ",\"labels\":\"" << b64(combined.data, combined.total()) << "\",\"observed\":[";
    for (int side = 0; side < 2; ++side) {
      o << (side ? "," : "") << '[';
      const auto & frags = geometry.observed_paths[side];
      for (size_t i = 0; i < frags.size(); ++i) {o << (i ? "," : ""); put_points(o, frags[i]);}
      o << ']';
    }
    o << "],\"centerline\":";
    put_points(o, center.points);
    o << ",\"support\":[";
    for (size_t i = 0; i < center.support.size(); ++i) o << (i ? "," : "") << int(center.support[i]);
    o << "],\"sample_limit\":" << (center.sample_limit_reached ? "true" : "false") << ",\"reference\":";
    put_points(o, reference);
    o << '}';
  } catch (const std::exception & e) {
    std::string msg = e.what();
    for (auto & c : msg) if (c == '"' || c == '\\') c = '\'';
    return "{\"error\":\"" + msg + "\"}";
  }
  return o.str();
}
}  // namespace

int main(int argc, char ** argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  if (mode == "infer" && argc == 5) return infer(argv[2], argv[3], argv[4]);
  if (mode == "serve") {
    std::string line;
    while (std::getline(std::cin, line)) std::cout << run(line) << std::endl;
    return 0;
  }
  std::cerr << "usage: lane_path_lab infer <model.onnx> <bev_dir> <cache_dir> | serve\n";
  return 2;
}
