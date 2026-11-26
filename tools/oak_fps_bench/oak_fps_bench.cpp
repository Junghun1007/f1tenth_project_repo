// OAK RGB·스테레오 수신 FPS/지연 진단 벤치마크 (ROS 없음, DepthAI v3 C++ SDK)
//
// camera_driver(1113ML)와 같은 장치 파이프라인을 설정만 하나씩 바꿔 가며 만들고,
// 각 설정에서 다음을 잰다.
//   - 요청 FPS / 센서 FPS(장치 순번·촬영 시각으로 계산) / 수신 FPS / 순번 누락
//   - 촬영(노출 중간) → 호스트 수신 지연 avg·p50·p95·max, 노출 시간을 뺀 지연
//   - RGB와 깊이 촬영 시각 차이 (센서 하드웨어 동기 여부 판단용)
//   - 장치 CPU(LeonCSS/LeonMSS)·메모리·온도 (SystemLogger), 호스트 CPU
// 같은 OAK를 쓰는 다른 프로그램(ros2 launch 등)은 모두 끄고 실행한다.
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/SystemInformation.hpp>
#include <depthai/pipeline/node/SystemLogger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#ifndef OAKBENCH_DEPTHAI_VERSION
#define OAKBENCH_DEPTHAI_VERSION "unknown"
#endif

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {

// ---------------------------------------------------------------- 설정
struct Config {
  std::string name;
  std::string why;               // 이 설정으로 무엇을 가르는지
  bool rgb{true};
  float rgb_fps{70.F};
  bool undistort{true};
  bool rgb_metadata_only{false}; // 장치는 똑같이 계산, 픽셀만 안 보냄 → 전송 vs 장치 연산
  bool depth{true};
  float depth_fps{30.F};
  bool align_rgb{true};
  bool lr_check{true};
  bool median3{true};
  bool imu{true};
  int imu_gyro_hz{400}, imu_accel_hz{400};   // camera_driver: 둘 다 400 Hz
  int imu_batch_threshold{1}, imu_max_batch{5};
  bool chunk_zero{true};         // camera_driver: setXLinkChunkSize(0)
  int rgb_exposure_us{0};        // 0 = 자동노출
  int ir_exposure_us{0};
  int iso{800};
  bool tunnel{false};            // camera_driver tunnel_enabled:=true (터널 밖 상태) 재현
  bool tunnel_800p{true};        // tunnel.ir_resolution 800p / 400p
  float dot{0.5F};
};

std::vector<Config> make_configs() {
  std::vector<Config> v;
  auto prod = [] { Config c; return c; };
  { Config c = prod(); c.name = "rgb_only"; c.why = "RGB 단독 (깊이 없음)"; c.depth = false; v.push_back(c); }
  { Config c = prod(); c.name = "rgb_only_noundist"; c.why = "RGB 단독, 장치 왜곡 보정 끔"; c.depth = false; c.undistort = false; v.push_back(c); }
  { Config c = prod(); c.name = "depth_only"; c.why = "깊이 단독 (RGB 없음, 정렬 끔 - 정렬 대상 없음)"; c.rgb = false; c.align_rgb = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod"; c.why = "실제 구성 (터널 꺼짐): RGB70+깊이30+정렬+IMU"; v.push_back(c); }
  { Config c = prod(); c.name = "prod_rgb_meta"; c.why = "실제 구성, RGB 픽셀 전송만 끔 (장치 연산은 동일)"; c.rgb_metadata_only = true; v.push_back(c); }
  { Config c = prod(); c.name = "prod_noundist"; c.why = "실제 구성, RGB 왜곡 보정 끔"; c.undistort = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod_noalign"; c.why = "실제 구성, 깊이 RGB 정렬 끔"; c.align_rgb = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod_nomedian"; c.why = "실제 구성, median 끔 (RGB 정렬은 LR check 필수라 유지)"; c.median3 = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod_noimu"; c.why = "실제 구성, IMU 끔"; c.imu = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod_imu_batch4"; c.why = "실제 구성, IMU 묶음 전송 4개(약 10 ms)"; c.imu_batch_threshold = 4; c.imu_max_batch = 10; v.push_back(c); }
  { Config c = prod(); c.name = "prod_imu_batch2"; c.why = "실제 구성, IMU 묶음 전송 2개(약 5 ms)"; c.imu_batch_threshold = 2; c.imu_max_batch = 10; v.push_back(c); }
  { Config c = prod(); c.name = "prod_imu_200"; c.why = "실제 구성, IMU 200 Hz"; c.imu_gyro_hz = 200; c.imu_accel_hz = 200; v.push_back(c); }
  { Config c = prod(); c.name = "prod_imu_g400_a100"; c.why = "실제 구성, 자이로 400 / 가속도 100 Hz"; c.imu_accel_hz = 100; v.push_back(c); }
  { Config c = prod(); c.name = "prod_chunkdefault"; c.why = "실제 구성, XLink chunk 기본값"; c.chunk_zero = false; v.push_back(c); }
  { Config c = prod(); c.name = "prod_fixed_exp"; c.why = "실제 구성, RGB·IR 수동 노출 3 ms"; c.rgb_exposure_us = 3000; c.ir_exposure_us = 3000; v.push_back(c); }
  { Config c = prod(); c.name = "ratio_60_30"; c.why = "RGB 60 / 깊이 30 (정수배)"; c.rgb_fps = 60.F; v.push_back(c); }
  { Config c = prod(); c.name = "ratio_70_35"; c.why = "RGB 70 / 깊이 35 (정수배)"; c.depth_fps = 35.F; v.push_back(c); }
  { Config c = prod(); c.name = "tunnel800_outside"; c.why = "tunnel_enabled, IR 800p, 터널 밖 상태 (스크립트 2개)"; c.tunnel = true; c.tunnel_800p = true; v.push_back(c); }
  { Config c = prod(); c.name = "tunnel400_outside"; c.why = "tunnel_enabled, IR 400p, 터널 밖 상태"; c.tunnel = true; c.tunnel_800p = false; v.push_back(c); }
  { Config c = prod(); c.name = "tunnel800_imu_batch4"; c.why = "tunnel_enabled, IR 800p, IMU 묶음 4개"; c.tunnel = true; c.imu_batch_threshold = 4; c.imu_max_batch = 10; v.push_back(c); }
  { Config c = prod(); c.name = "tunnel400_imu_batch4"; c.why = "tunnel_enabled, IR 400p, IMU 묶음 4개"; c.tunnel = true; c.tunnel_800p = false; c.imu_batch_threshold = 4; c.imu_max_batch = 10; v.push_back(c); }
  return v;
}

// ---------------------------------------------------------------- 통계
double pct(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  const double idx = p / 100.0 * double(v.size() - 1);
  const auto lo = std::size_t(std::floor(idx)), hi = std::size_t(std::ceil(idx));
  return v[lo] + (v[hi] - v[lo]) * (idx - double(lo));
}
double mean(const std::vector<double> & v) { return v.empty() ? NAN : std::accumulate(v.begin(), v.end(), 0.0) / double(v.size()); }

struct StreamStats {
  std::mutex m;
  bool measuring{false};
  std::vector<double> latency_ms, latency_end_ms, exposure_ms, interval_ms;
  std::vector<double> iso;
  std::int64_t first_seq{-1}, last_seq{-1};
  Clock::time_point first_ts{}, last_ts{}, last_rx{};
  std::uint64_t count{0}, gaps{0}, gap_events{0}, bad{0};
  void reset() {
    latency_ms.clear(); latency_end_ms.clear(); exposure_ms.clear(); interval_ms.clear(); iso.clear();
    first_seq = last_seq = -1; first_ts = last_ts = last_rx = {}; count = gaps = gap_events = bad = 0;
  }
  void add(const dai::ImgFrame & f, Clock::time_point rx) {
    std::lock_guard<std::mutex> l(m);
    if (!measuring) return;
    const auto ts = f.getTimestamp(dai::CameraExposureOffset::MIDDLE);
    const auto seq = f.getSequenceNum();
    const double exp_ms = std::chrono::duration<double, std::milli>(f.getExposureTime()).count();
    const double lat = std::chrono::duration<double, std::milli>(rx - ts).count();
    if (first_seq < 0) { first_seq = seq; first_ts = ts; }
    else {
      if (seq > last_seq + 1) { gaps += std::uint64_t(seq - last_seq - 1); ++gap_events; }
      if (seq <= last_seq) ++bad;
      interval_ms.push_back(std::chrono::duration<double, std::milli>(rx - last_rx).count());
    }
    last_seq = std::max(last_seq, seq); last_ts = ts; last_rx = rx;
    ++count;
    latency_ms.push_back(lat);
    latency_end_ms.push_back(lat - exp_ms / 2.0);   // 노출 끝 기준 (노출 길이 영향 제거)
    exposure_ms.push_back(exp_ms);
    iso.push_back(double(f.getSensitivity()));
  }
  double sensor_fps() const {   // 장치가 실제로 찍은 속도 (중간에 버려진 프레임 포함)
    const double dt = std::chrono::duration<double>(last_ts - first_ts).count();
    return (count >= 2 && dt > 0) ? double(last_seq - first_seq) / dt : NAN;
  }
};

struct SysStats {
  std::mutex m;
  bool measuring{false};
  std::vector<double> css, mss, ddr_used_mb, cmx_used_kb, temp;
  void reset() { css.clear(); mss.clear(); ddr_used_mb.clear(); cmx_used_kb.clear(); temp.clear(); }
};

struct HostCpu {   // /proc 기반 (이 프로세스, 시스템 전체)
  long long proc_ticks{0}, busy{0}, total{0};
  Clock::time_point t{};
  static HostCpu sample() {
    HostCpu h;
    h.t = Clock::now();
    std::ifstream s("/proc/self/stat");
    std::string line; std::getline(s, line);
    const auto rp = line.rfind(')');
    if (rp != std::string::npos) {
      std::istringstream ss(line.substr(rp + 2));
      std::string tok; long long ut = 0, st = 0;
      for (int field = 3; ss >> tok; ++field) { if (field == 14) ut = std::stoll(tok); if (field == 15) { st = std::stoll(tok); break; } }
      h.proc_ticks = ut + st;
    }
    std::ifstream p("/proc/stat");
    std::string cpu; p >> cpu;
    std::vector<long long> f; long long x;
    while (f.size() < 8 && p >> x) f.push_back(x);
    for (auto y : f) h.total += y;
    if (f.size() >= 5) h.busy = h.total - f[3] - f[4];
    return h;
  }
};

// ---------------------------------------------------------------- 장치 스크립트 (camera_driver와 같은 내용)
const char * kStereoGateScript = R"PY(
active = True
left = None
right = None
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        active = bool(control.getData()[0])
    if left is None:
        left = node.io['left'].get()
    if right is None:
        right = node.io['right'].get()
    left_seq = left.getSequenceNum()
    right_seq = right.getSequenceNum()
    if left_seq < right_seq:
        left = None
    elif right_seq < left_seq:
        right = None
    else:
        if active:
            node.io['left_out'].send(left)
            node.io['right_out'].send(right)
        left = None
        right = None
)PY";

const char * kIrGateScript = R"PY(
import time

route = 0
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        route = control.getData()[0]
    if route == 2:
        frame = node.io['low'].tryGet()
        if frame is None:
            time.sleep(0.002)
            continue
    else:
        frame = node.io['image'].get()
    if route != 0:
        node.io['out'].send(frame)
)PY";

void latest_only(dai::Node::Output * out, bool metadata_only = false) {
  if (!out) return;
  const auto bridge = out->getXLinkBridge();
  if (!bridge || !bridge->xLinkOut) { std::fprintf(stderr, "  (warn) XLink bridge missing\n"); return; }
  bridge->xLinkOut->input.setMaxSize(1);
  bridge->xLinkOut->input.setBlocking(false);
  if (metadata_only) bridge->xLinkOut->setMetadataOnly(true);
}

std::shared_ptr<dai::Device> open_device() {
  for (int attempt = 1; attempt <= 30; ++attempt) {
    try {
      return std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    } catch (const std::exception & e) {
      if (attempt == 30) throw;
      std::this_thread::sleep_for(1s);   // 이전 설정 종료 후 장치 재부팅 대기
    }
  }
  return nullptr;
}

const char * usb_name(dai::UsbSpeed s) {
  switch (s) {
    case dai::UsbSpeed::LOW: return "LOW(USB1)";
    case dai::UsbSpeed::FULL: return "FULL(USB1)";
    case dai::UsbSpeed::HIGH: return "HIGH(USB2 480M)";
    case dai::UsbSpeed::SUPER: return "SUPER(USB3 5G)";
    case dai::UsbSpeed::SUPER_PLUS: return "SUPER_PLUS(USB3 10G)";
    default: return "UNKNOWN";
  }
}

struct Result {
  Config c;
  std::string error;
  std::string usb;
  double rgb_rx_fps{NAN}, rgb_sensor_fps{NAN}, depth_rx_fps{NAN}, depth_sensor_fps{NAN};
  std::uint64_t rgb_gaps{0}, depth_gaps{0}, rgb_count{0}, depth_count{0};
  double rgb_lat_avg{NAN}, rgb_lat_p50{NAN}, rgb_lat_p95{NAN}, rgb_lat_max{NAN}, rgb_lat_end_p50{NAN}, rgb_exp{NAN}, rgb_iso{NAN};
  double depth_lat_avg{NAN}, depth_lat_p50{NAN}, depth_lat_p95{NAN}, depth_lat_max{NAN}, depth_lat_end_p50{NAN}, depth_exp{NAN};
  double rgb_int_p95{NAN}, depth_int_p95{NAN};
  double sync_offset_p50{NAN}, sync_offset_p95{NAN};
  double css{NAN}, mss{NAN}, ddr{NAN}, cmx{NAN}, temp{NAN};
  double host_proc_cpu{NAN}, host_sys_cpu{NAN};
  double imu_hz{NAN}, imu_age_p50{NAN}, imu_age_p95{NAN}, imu_age_max{NAN};
};

// ---------------------------------------------------------------- 한 설정 실행
Result run(const Config & c, double warmup_s, double measure_s, bool verbose_once) {
  Result r; r.c = c;
  std::shared_ptr<dai::Device> device;
  std::unique_ptr<dai::Pipeline> pipeline;
  try {
    device = open_device();
    r.usb = usb_name(device->getUsbSpeed());
    if (verbose_once) {   // 정보 출력 실패는 측정과 무관하므로 무시
      std::printf("[device] usb=%s\n", r.usb.c_str());
      try { std::printf("[device] name=%s\n", device->getDeviceName().c_str()); } catch (const std::exception & e) { std::printf("[device] name=? (%s)\n", e.what()); }
      try { std::printf("[device] product=%s\n", device->getProductName().c_str()); } catch (const std::exception & e) { std::printf("[device] product=? (%s)\n", e.what()); }
      try {
        for (const auto & f : device->getConnectedCameraFeatures())
          std::printf("[device] camera socket=%d sensor=%s max=%dx%d\n", int(f.socket), f.sensorName.c_str(), f.width, f.height);
      } catch (const std::exception & e) { std::printf("[device] cameras=? (%s)\n", e.what()); }
      try { std::printf("[device] imu=%s\n", device->getConnectedIMU().c_str()); } catch (const std::exception & e) { std::printf("[device] imu=? (%s)\n", e.what()); }
      std::fflush(stdout);
    }
    pipeline = std::make_unique<dai::Pipeline>(device);
    pipeline->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    if (c.chunk_zero) pipeline->setXLinkChunkSize(0);

    // ---- 스테레오 깊이 (point_cloud::addStereoDepth와 같은 설정)
    dai::Node::Output * depth_out = nullptr;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<dai::node::Camera> left, right;
    std::vector<dai::Node::Output *> meta_outputs;   // 진단용 160x96 (metadata only)
    dai::Node::Output * ir_gate_out = nullptr;
    std::vector<std::shared_ptr<dai::InputQueue>> keep_inputs;
    if (c.depth) {
      const std::pair<std::uint32_t, std::uint32_t> depth_size{640U, 400U};
      const std::pair<std::uint32_t, std::uint32_t> sensor = (c.tunnel && c.tunnel_800p) ? std::make_pair(1280U, 800U) : depth_size;
      left = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, sensor, c.depth_fps);
      right = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, sensor, c.depth_fps);
      if (c.ir_exposure_us > 0) {
        left->initialControl.setManualExposure(std::uint32_t(c.ir_exposure_us), std::uint32_t(c.iso));
        right->initialControl.setManualExposure(std::uint32_t(c.ir_exposure_us), std::uint32_t(c.iso));
      }
      auto * lo = left->requestOutput(depth_size);
      auto * ro = right->requestOutput(depth_size);
      if (c.tunnel) {
        auto gate = pipeline->create<dai::node::Script>();
        gate->setScript(kStereoGateScript);
        lo->link(gate->inputs["left"]);
        ro->link(gate->inputs["right"]);
        for (const auto * n : {"left", "right", "control"}) { gate->inputs[n].setMaxSize(1); gate->inputs[n].setBlocking(false); }
        keep_inputs.push_back(gate->inputs["control"].createInputQueue());
        lo = &gate->outputs["left_out"];
        ro = &gate->outputs["right_out"];
      }
      stereo = pipeline->create<dai::node::StereoDepth>();
      stereo->build(*lo, *ro, dai::node::StereoDepth::PresetMode::FAST_DENSITY);
      stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
      if (c.align_rgb) { stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A); stereo->setOutputSize(640, 400); }
      else stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
      stereo->initialConfig->setConfidenceThreshold(200);
      stereo->initialConfig->setLeftRightCheckThreshold(10);
      stereo->setLeftRightCheck(c.lr_check);
      stereo->setSubpixel(false);
      stereo->setExtendedDisparity(false);
      stereo->initialConfig->setMedianFilter(c.median3 ? dai::StereoDepthConfig::MedianFilter::KERNEL_3x3
                                                       : dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF);
      auto & post = stereo->initialConfig->postProcessing;
      post.decimationFilter.decimationFactor = 1;
      post.temporalFilter.enable = false;
      post.spatialFilter.enable = false;
      post.speckleFilter.enable = false;
      post.holeFilling.enable = false;
      post.adaptiveMedianFilter.enable = false;
      post.bilateralSigmaValue = 0;
      post.thresholdFilter.minRange = 0;
      post.thresholdFilter.maxRange = 65535;
      depth_out = &stereo->depth;

      if (c.tunnel) {   // camera_driver 터널 IR 경로 (터널 밖: route 0 → 호스트로 안 보냄)
        for (auto & cam : {left, right}) meta_outputs.push_back(cam->requestOutput({160U, 96U}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 5.0F, false));
        const std::pair<std::uint32_t, std::uint32_t> ir_size = c.tunnel_800p ? std::make_pair(1280U, 800U) : depth_size;
        auto * ir = left->requestOutput(ir_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.0F, false);
        auto gate = pipeline->create<dai::node::Script>();
        gate->setScript(kIrGateScript);
        ir->link(gate->inputs["image"]);
        if (c.tunnel_800p) {
          auto * low = left->requestOutput(depth_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.0F, false);
          low->link(gate->inputs["low"]);
          gate->inputs["low"].setMaxSize(1); gate->inputs["low"].setBlocking(false);
        }
        for (const auto * n : {"image", "control"}) { gate->inputs[n].setMaxSize(1); gate->inputs[n].setBlocking(false); }
        keep_inputs.push_back(gate->inputs["control"].createInputQueue());
        ir_gate_out = &gate->outputs["out"];
      }
    }

    // ---- RGB (camera_driver와 같은 요청)
    dai::Node::Output * rgb_out = nullptr;
    if (c.rgb) {
      auto cam = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::make_pair(1280U, 800U), c.rgb_fps);
      if (c.rgb_exposure_us > 0) cam->initialControl.setManualExposure(std::uint32_t(c.rgb_exposure_us), std::uint32_t(c.iso));
      else cam->initialControl.setAutoExposureEnable();
      rgb_out = cam->requestOutput(std::make_pair(1280U, 800U), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, c.rgb_fps, c.undistort);
    }

    std::shared_ptr<dai::MessageQueue> rgb_q, depth_q, imu_q, sys_q, ir_q;
    std::vector<std::shared_ptr<dai::MessageQueue>> meta_q;
    if (rgb_out) rgb_q = rgb_out->createOutputQueue(1, false);
    if (depth_out) depth_q = depth_out->createOutputQueue(1, false);
    if (ir_gate_out) ir_q = ir_gate_out->createOutputQueue(1, false);
    for (auto * o : meta_outputs) meta_q.push_back(o->createOutputQueue(1, false));
    if (c.imu) {
      auto imu = pipeline->create<dai::node::IMU>();
      imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_CALIBRATED, std::uint32_t(c.imu_accel_hz));
      imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, std::uint32_t(c.imu_gyro_hz));
      imu->setBatchReportThreshold(c.imu_batch_threshold);
      imu->setMaxBatchReports(c.imu_max_batch);
      imu_q = imu->out.createOutputQueue(80, false);
    }
    auto sys = pipeline->create<dai::node::SystemLogger>();
    sys->setRate(2.0F);
    sys_q = sys->out.createOutputQueue(4, false);

    pipeline->build();
    latest_only(rgb_out, c.rgb_metadata_only);
    latest_only(depth_out);
    latest_only(ir_gate_out);
    for (auto * o : meta_outputs) latest_only(o, true);
    pipeline->start();
    if (c.depth) {
      device->setIrLaserDotProjectorIntensity(c.dot);
      device->setIrFloodLightIntensity(0.0F);
    }

    // ---- 수신 스레드 (스트림별로 따로: 한쪽이 다른 쪽을 막지 않게)
    StreamStats rgb_s, depth_s;
    SysStats sys_s;
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> imu_packets{0};
    std::atomic<bool> imu_measuring{false};
    std::mutex imu_m;
    std::vector<double> imu_age_ms;   // 자이로 측정 → 호스트 수신 (영상 안정화는 5 ms 대기·12 ms 나이 한계)
    std::mutex sync_m;
    std::deque<Clock::time_point> recent_rgb;
    std::vector<double> sync_offsets;
    std::atomic<bool> sync_measuring{false};
    std::vector<std::thread> threads;
    if (rgb_q) threads.emplace_back([&] {
      while (!stop) {
        auto f = rgb_q->tryGet<dai::ImgFrame>();
        if (!f) { std::this_thread::sleep_for(100us); continue; }
        const auto rx = Clock::now();
        rgb_s.add(*f, rx);
        std::lock_guard<std::mutex> l(sync_m);
        recent_rgb.push_back(f->getTimestamp(dai::CameraExposureOffset::START));
        while (recent_rgb.size() > 32) recent_rgb.pop_front();
      }
    });
    if (depth_q) threads.emplace_back([&] {
      while (!stop) {
        auto f = depth_q->tryGet<dai::ImgFrame>();
        if (!f) { std::this_thread::sleep_for(100us); continue; }
        const auto rx = Clock::now();
        depth_s.add(*f, rx);
        if (!sync_measuring) continue;
        const auto t = f->getTimestamp(dai::CameraExposureOffset::START);
        std::lock_guard<std::mutex> l(sync_m);
        double best = 1e9;
        for (const auto & q : recent_rgb) best = std::min(best, std::abs(std::chrono::duration<double, std::milli>(t - q).count()));
        if (best < 1e8) sync_offsets.push_back(best);
      }
    });
    if (imu_q) threads.emplace_back([&] {
      while (!stop) {
        auto d = imu_q->tryGet<dai::IMUData>();
        if (!d) { std::this_thread::sleep_for(200us); continue; }
        if (!imu_measuring) continue;
        const auto rx = Clock::now();
        imu_packets += d->packets.size();
        std::lock_guard<std::mutex> l(imu_m);
        for (const auto & p : d->packets) imu_age_ms.push_back(std::chrono::duration<double, std::milli>(rx - p.gyroscope.getTimestamp()).count());
      }
    });
    if (ir_q || !meta_q.empty()) threads.emplace_back([&] {   // 터널 출력 비우기 (camera_driver처럼 꺼내 감)
      while (!stop) {
        bool any = false;
        if (ir_q && ir_q->tryGet<dai::ImgFrame>()) any = true;
        for (auto & q : meta_q) if (q->tryGet<dai::ImgFrame>()) any = true;
        if (!any) std::this_thread::sleep_for(1ms);
      }
    });
    threads.emplace_back([&] {
      while (!stop) {
        auto s = sys_q->tryGet<dai::SystemInformation>();
        if (!s) { std::this_thread::sleep_for(20ms); continue; }
        std::lock_guard<std::mutex> l(sys_s.m);
        if (!sys_s.measuring) continue;
        sys_s.css.push_back(s->leonCssCpuUsage.average * 100.0);
        sys_s.mss.push_back(s->leonMssCpuUsage.average * 100.0);
        sys_s.ddr_used_mb.push_back(double(s->ddrMemoryUsage.used) / 1048576.0);
        sys_s.cmx_used_kb.push_back(double(s->cmxMemoryUsage.used) / 1024.0);
        sys_s.temp.push_back(s->chipTemperature.average);
      }
    });

    std::this_thread::sleep_for(std::chrono::duration<double>(warmup_s));
    for (auto * s : {&rgb_s, &depth_s}) { std::lock_guard<std::mutex> l(s->m); s->reset(); s->measuring = true; }
    { std::lock_guard<std::mutex> l(sys_s.m); sys_s.reset(); sys_s.measuring = true; }
    { std::lock_guard<std::mutex> l(sync_m); sync_offsets.clear(); }
    sync_measuring = true;
    imu_measuring = true;
    const auto h0 = HostCpu::sample();
    std::this_thread::sleep_for(std::chrono::duration<double>(measure_s));
    const auto h1 = HostCpu::sample();
    for (auto * s : {&rgb_s, &depth_s}) { std::lock_guard<std::mutex> l(s->m); s->measuring = false; }
    { std::lock_guard<std::mutex> l(sys_s.m); sys_s.measuring = false; }
    sync_measuring = false;
    imu_measuring = false;
    stop = true;
    for (auto & t : threads) t.join();

    const double tick = double(sysconf(_SC_CLK_TCK));
    const double wall = std::chrono::duration<double>(h1.t - h0.t).count();
    r.host_proc_cpu = 100.0 * double(h1.proc_ticks - h0.proc_ticks) / tick / wall;   // 100 = 코어 1개
    if (h1.total > h0.total) r.host_sys_cpu = 100.0 * double(h1.busy - h0.busy) / double(h1.total - h0.total);
    r.imu_hz = double(imu_packets.load()) / measure_s;
    { std::lock_guard<std::mutex> l(imu_m); r.imu_age_p50 = pct(imu_age_ms, 50); r.imu_age_p95 = pct(imu_age_ms, 95);
      r.imu_age_max = imu_age_ms.empty() ? NAN : *std::max_element(imu_age_ms.begin(), imu_age_ms.end()); }
    auto fill = [&](StreamStats & s, double & rx, double & sensor, std::uint64_t & gaps, std::uint64_t & count, double & avg, double & p50,
                    double & p95, double & mx, double & end50, double & exp, double & int95) {
      std::lock_guard<std::mutex> l(s.m);
      rx = double(s.count) / measure_s;
      sensor = s.sensor_fps();
      gaps = s.gaps; count = s.count;
      avg = mean(s.latency_ms); p50 = pct(s.latency_ms, 50); p95 = pct(s.latency_ms, 95);
      mx = s.latency_ms.empty() ? NAN : *std::max_element(s.latency_ms.begin(), s.latency_ms.end());
      end50 = pct(s.latency_end_ms, 50);
      exp = mean(s.exposure_ms);
      int95 = pct(s.interval_ms, 95);
    };
    if (c.rgb) {
      fill(rgb_s, r.rgb_rx_fps, r.rgb_sensor_fps, r.rgb_gaps, r.rgb_count, r.rgb_lat_avg, r.rgb_lat_p50, r.rgb_lat_p95, r.rgb_lat_max,
           r.rgb_lat_end_p50, r.rgb_exp, r.rgb_int_p95);
      std::lock_guard<std::mutex> l(rgb_s.m);
      r.rgb_iso = mean(rgb_s.iso);
    }
    if (c.depth)
      fill(depth_s, r.depth_rx_fps, r.depth_sensor_fps, r.depth_gaps, r.depth_count, r.depth_lat_avg, r.depth_lat_p50, r.depth_lat_p95,
           r.depth_lat_max, r.depth_lat_end_p50, r.depth_exp, r.depth_int_p95);
    { std::lock_guard<std::mutex> l(sync_m); r.sync_offset_p50 = pct(sync_offsets, 50); r.sync_offset_p95 = pct(sync_offsets, 95); }
    { std::lock_guard<std::mutex> l(sys_s.m);
      r.css = mean(sys_s.css); r.mss = mean(sys_s.mss); r.ddr = mean(sys_s.ddr_used_mb); r.cmx = mean(sys_s.cmx_used_kb); r.temp = mean(sys_s.temp); }
    pipeline->stop();
  } catch (const std::exception & e) {
    r.error = e.what();
  }
  pipeline.reset();
  if (device) { try { device->close(); } catch (...) {} }
  device.reset();
  std::this_thread::sleep_for(1500ms);
  return r;
}

std::string f1(double v, int prec = 1) {
  if (!std::isfinite(v)) return "-";
  char b[32]; std::snprintf(b, sizeof(b), "%.*f", prec, v); return b;
}

void print_result(const Result & r, std::FILE * out) {
  const auto & c = r.c;
  std::fprintf(out, "\n=== %s : %s\n", c.name.c_str(), c.why.c_str());
  if (!r.error.empty()) { std::fprintf(out, "  ERROR: %s\n", r.error.c_str()); return; }
  std::fprintf(out, "  usb=%s  device CPU css=%s%% mss=%s%%  ddr=%sMB cmx=%sKB temp=%sC  host proc=%s%%(1core=100) sys=%s%%\n",
               r.usb.c_str(), f1(r.css).c_str(), f1(r.mss).c_str(), f1(r.ddr).c_str(), f1(r.cmx, 0).c_str(), f1(r.temp).c_str(),
               f1(r.host_proc_cpu).c_str(), f1(r.host_sys_cpu).c_str());
  if (c.imu)
    std::fprintf(out, "  IMU   gyro=%dHz accel=%dHz batch=%d  rx=%s packets/s  age p50/p95/max=%s/%s/%s ms\n", c.imu_gyro_hz, c.imu_accel_hz,
                 c.imu_batch_threshold, f1(r.imu_hz, 0).c_str(), f1(r.imu_age_p50, 2).c_str(), f1(r.imu_age_p95, 2).c_str(), f1(r.imu_age_max, 2).c_str());
  if (c.rgb)
    std::fprintf(out, "  RGB   req=%s sensor=%s rx=%s fps  gaps=%llu/%llu  lat avg/p50/p95/max=%s/%s/%s/%s ms  lat_end_p50=%s ms  exp=%s ms iso=%s  rx_int_p95=%s ms%s\n",
                 f1(c.rgb_fps).c_str(), f1(r.rgb_sensor_fps).c_str(), f1(r.rgb_rx_fps).c_str(), (unsigned long long)r.rgb_gaps,
                 (unsigned long long)(r.rgb_gaps + r.rgb_count), f1(r.rgb_lat_avg).c_str(), f1(r.rgb_lat_p50).c_str(), f1(r.rgb_lat_p95).c_str(),
                 f1(r.rgb_lat_max).c_str(), f1(r.rgb_lat_end_p50).c_str(), f1(r.rgb_exp, 2).c_str(), f1(r.rgb_iso, 0).c_str(),
                 f1(r.rgb_int_p95).c_str(), c.rgb_metadata_only ? "  (metadata only)" : "");
  if (c.depth)
    std::fprintf(out, "  DEPTH req=%s sensor=%s rx=%s fps  gaps=%llu/%llu  lat avg/p50/p95/max=%s/%s/%s/%s ms  lat_end_p50=%s ms  exp=%s ms  rx_int_p95=%s ms\n",
                 f1(c.depth_fps).c_str(), f1(r.depth_sensor_fps).c_str(), f1(r.depth_rx_fps).c_str(), (unsigned long long)r.depth_gaps,
                 (unsigned long long)(r.depth_gaps + r.depth_count), f1(r.depth_lat_avg).c_str(), f1(r.depth_lat_p50).c_str(),
                 f1(r.depth_lat_p95).c_str(), f1(r.depth_lat_max).c_str(), f1(r.depth_lat_end_p50).c_str(), f1(r.depth_exp, 2).c_str(),
                 f1(r.depth_int_p95).c_str());
  if (c.rgb && c.depth)
    std::fprintf(out, "  SYNC  |depth start - nearest rgb start| p50/p95 = %s/%s ms (0에 붙어 있으면 센서 하드웨어 동기)\n",
                 f1(r.sync_offset_p50, 2).c_str(), f1(r.sync_offset_p95, 2).c_str());
  std::fflush(out);
}

}  // namespace

int main(int argc, char ** argv) {
  double warmup = 4.0, measure = 12.0;
  std::string only, csv_path = "oak_fps_bench.csv";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&] { if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", a.c_str()); std::exit(2); } return std::string(argv[++i]); };
    if (a == "--warmup") warmup = std::stod(next());
    else if (a == "--seconds") measure = std::stod(next());
    else if (a == "--only") only = next();
    else if (a == "--csv") csv_path = next();
    else if (a == "--list") { for (const auto & c : make_configs()) std::printf("%-20s %s\n", c.name.c_str(), c.why.c_str()); return 0; }
    else { std::fprintf(stderr, "usage: %s [--seconds 12] [--warmup 4] [--only name1,name2] [--csv path] [--list]\n", argv[0]); return 2; }
  }
  auto configs = make_configs();
  if (!only.empty()) {
    std::vector<Config> keep;
    std::stringstream ss(only); std::string n;
    while (std::getline(ss, n, ','))
      for (const auto & c : configs) if (c.name == n) keep.push_back(c);
    configs = keep;
  }
  std::printf("[bench] depthai=%s configs=%zu warmup=%.1fs measure=%.1fs\n", OAKBENCH_DEPTHAI_VERSION, configs.size(), warmup, measure);
  std::vector<Result> results;
  bool first = true;
  for (const auto & c : configs) {
    std::printf("[bench] running %s ...\n", c.name.c_str()); std::fflush(stdout);
    results.push_back(run(c, warmup, measure, first));
    first = false;
    print_result(results.back(), stdout);
  }
  std::ofstream csv(csv_path);
  csv << "name,usb,rgb_req,rgb_sensor,rgb_rx,rgb_gaps,rgb_lat_avg,rgb_lat_p50,rgb_lat_p95,rgb_lat_max,rgb_lat_end_p50,rgb_exp_ms,rgb_iso,"
         "depth_req,depth_sensor,depth_rx,depth_gaps,depth_lat_avg,depth_lat_p50,depth_lat_p95,depth_lat_max,depth_lat_end_p50,depth_exp_ms,"
         "sync_p50,sync_p95,css,mss,ddr_mb,cmx_kb,temp,host_proc_cpu,host_sys_cpu,imu_hz,imu_age_p50,imu_age_p95,imu_age_max,error\n";
  for (const auto & r : results)
    csv << r.c.name << ',' << r.usb << ',' << r.c.rgb_fps << ',' << f1(r.rgb_sensor_fps) << ',' << f1(r.rgb_rx_fps) << ',' << r.rgb_gaps << ','
        << f1(r.rgb_lat_avg) << ',' << f1(r.rgb_lat_p50) << ',' << f1(r.rgb_lat_p95) << ',' << f1(r.rgb_lat_max) << ',' << f1(r.rgb_lat_end_p50) << ','
        << f1(r.rgb_exp, 2) << ',' << f1(r.rgb_iso, 0) << ',' << r.c.depth_fps << ',' << f1(r.depth_sensor_fps) << ',' << f1(r.depth_rx_fps) << ','
        << r.depth_gaps << ',' << f1(r.depth_lat_avg) << ',' << f1(r.depth_lat_p50) << ',' << f1(r.depth_lat_p95) << ',' << f1(r.depth_lat_max) << ','
        << f1(r.depth_lat_end_p50) << ',' << f1(r.depth_exp, 2) << ',' << f1(r.sync_offset_p50, 2) << ',' << f1(r.sync_offset_p95, 2) << ','
        << f1(r.css) << ',' << f1(r.mss) << ',' << f1(r.ddr) << ',' << f1(r.cmx, 0) << ',' << f1(r.temp) << ',' << f1(r.host_proc_cpu) << ','
        << f1(r.host_sys_cpu) << ',' << f1(r.imu_hz, 0) << ',' << f1(r.imu_age_p50, 2) << ',' << f1(r.imu_age_p95, 2) << ','
        << f1(r.imu_age_max, 2) << ",\"" << r.error << "\"\n";
  std::printf("\n[bench] done. csv=%s\n", csv_path.c_str());
  return 0;
}
