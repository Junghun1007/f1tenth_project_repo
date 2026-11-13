// Disposable, standalone DepthAI v3 acquisition probe. No ROS or actuator interfaces.
#include <depthai/depthai.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void handleSignal(int) {interrupted = 1;}

struct Frame {
    double elapsed, latency;
    std::int64_t sequence, capturedNs;
    unsigned width, height;
};

static Json distribution(std::vector<double> values) {
    if(values.empty()) return nullptr;
    std::sort(values.begin(), values.end());
    return {{"mean", std::accumulate(values.begin(), values.end(), 0.) / values.size()},
            {"p95", values[static_cast<std::size_t>(std::ceil(values.size() * .95)) - 1]},
            {"max", values.back()}};
}

static Json frameStats(const std::vector<Frame>& frames, double seconds) {
    std::vector<double> latency, intervals;
    std::int64_t gaps = 0, duplicates = 0;
    for(std::size_t i = 0; i < frames.size(); ++i) {
        latency.push_back(frames[i].latency);
        if(i == 0) continue;
        const auto delta = frames[i].sequence - frames[i - 1].sequence;
        if(delta > 1) gaps += delta - 1;
        if(delta <= 0) ++duplicates;
        intervals.push_back((frames[i].capturedNs - frames[i - 1].capturedNs) / 1.e6);
    }
    return {{"count", frames.size()}, {"receive_hz", frames.size() / seconds},
            {"sequence_gaps", gaps}, {"nonincreasing_sequences", duplicates},
            {"sensor_to_host_ms", distribution(latency)}, {"capture_interval_ms", distribution(intervals)}};
}

static void saveFrames(const std::string& path, const std::vector<Frame>& frames) {
    std::ofstream out(path);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << "elapsed_sec,sequence,capture_steady_ns,sensor_to_host_ms,width,height\n";
    out.precision(12);
    for(const auto& f : frames)
        out << f.elapsed << ',' << f.sequence << ',' << f.capturedNs << ',' << f.latency
            << ',' << f.width << ',' << f.height << '\n';
}

static void latestOnly(dai::Node::Output& output) {
    const auto bridge = output.getXLinkBridge();
    if(!bridge || !bridge->xLinkOut) throw std::runtime_error("Missing XLink bridge");
    bridge->xLinkOut->input.setMaxSize(1);
    bridge->xLinkOut->input.setBlocking(false);
}

static Json run(const Json& config, const std::string& directory) {
    const bool stereoEnabled = config.at("stereo");
    const unsigned width = config.at("width"), height = config.at("height");
    const double duration = config.at("duration_sec"), warmup = config.at("warmup_sec");
    if(duration < 1 || duration > 300 || warmup < 0 || warmup > 120)
        throw std::invalid_argument("Invalid measurement duration");
    if(!((width == 640 && height == 400) || (width == 512 && height == 320) || (width == 320 && height == 200)))
        throw std::invalid_argument("Unsupported experimental size");
    const auto median = config.at("median").get<std::string>();
    if(median != "off" && median != "3x3") throw std::invalid_argument("Invalid median filter");
    const unsigned cropHeight = height * 65 / 100, cropTop = height - cropHeight;
    auto device = config.value("device_id", std::string{}).empty()
        ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER)
        : std::make_shared<dai::Device>(dai::DeviceInfo(config.at("device_id").get<std::string>()), dai::UsbSpeed::SUPER);
    if(device->getUsbSpeed() != dai::UsbSpeed::SUPER && device->getUsbSpeed() != dai::UsbSpeed::SUPER_PLUS)
        throw std::runtime_error("USB SuperSpeed required for comparable RGB 80 FPS results");
    dai::Pipeline pipeline(device);
    pipeline.setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline.setXLinkChunkSize(0);
    auto rgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A, std::make_pair(1280U, 800U), 80.F);
    auto* rgbOut = rgb->requestOutput({1280, 800}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 80.F, true);
    auto rgbQueue = rgbOut->createOutputQueue(1, false);
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<dai::MessageQueue> depthQueue;
    if(stereoEnabled) {
        // All cases use the same 640x400 sensor mode. Only the ISP output size changes.
        // Crop BOTH input images before disparity matching, with ImgTransformations preserved.
        std::vector<std::shared_ptr<dai::node::ImageManip>> crops;
        for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
            auto camera = pipeline.create<dai::node::Camera>()->build(socket, std::make_pair(640U, 400U), 40.F);
            auto* output = camera->requestOutput({width, height}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 40.F, false);
            auto crop = pipeline.create<dai::node::ImageManip>();
            crop->setRunOnHost(false);
            crop->initialConfig->addCrop(0, cropTop, width, cropHeight);
            crop->initialConfig->setFrameType(dai::ImgFrame::Type::GRAY8);
            crop->setMaxOutputFrameSize(width * cropHeight);
            output->link(crop->inputImage);
            crops.push_back(crop);
        }
        stereo = pipeline.create<dai::node::StereoDepth>();
        stereo->build(crops[0]->out, crops[1]->out, dai::node::StereoDepth::PresetMode::FAST_DENSITY);
        stereo->setInputResolution(width, cropHeight);
        stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
        stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
        stereo->initialConfig->setConfidenceThreshold(200);
        stereo->initialConfig->setLeftRightCheckThreshold(10);
        stereo->setLeftRightCheck(config.at("lr_check").get<bool>());
        stereo->setSubpixel(false);
        stereo->setExtendedDisparity(false);
        stereo->initialConfig->setMedianFilter(median == "off" ? dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF
                                                              : dai::StereoDepthConfig::MedianFilter::KERNEL_3x3);
        auto& post = stereo->initialConfig->postProcessing;
        post.decimationFilter.decimationFactor = 1;
        post.temporalFilter.enable = false;
        post.spatialFilter.enable = false;
        post.speckleFilter.enable = false;
        post.holeFilling.enable = false;
        post.adaptiveMedianFilter.enable = false;
        post.bilateralSigmaValue = 0;
        post.thresholdFilter.minRange = 100;
        post.thresholdFilter.maxRange = 3000;
        depthQueue = stereo->depth.createOutputQueue(1, false);
    }
    auto imu = pipeline.create<dai::node::IMU>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_CALIBRATED, dai::IMUSensor::GYROSCOPE_CALIBRATED}, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(5);
    auto imuQueue = imu->out.createOutputQueue(80, false);
    pipeline.build();
    latestOnly(*rgbOut);
    latestOnly(imu->out);
    if(stereo) latestOnly(stereo->depth);
    pipeline.start();
    if(stereoEnabled) {
        const double intensity = config.at("dot_intensity");
        if(!device->setIrLaserDotProjectorIntensity(intensity) && intensity > 0)
            throw std::runtime_error("IR projector unsupported; use --dot-intensity 0 for ALL cases");
        device->setIrFloodLightIntensity(0);
    }
    const auto firstDeadline = Clock::now() + std::chrono::seconds(15);
    // Require real RGB/depth before starting the common warmup and measurement clocks.
    bool seenRgb = false, seenDepth = !stereoEnabled;
    while(!interrupted && (!seenRgb || !seenDepth) && Clock::now() < firstDeadline) {
        seenRgb = bool(rgbQueue->tryGet<dai::ImgFrame>()) || seenRgb;
        if(depthQueue) seenDepth = bool(depthQueue->tryGet<dai::ImgFrame>()) || seenDepth;
        imuQueue->tryGet<dai::IMUData>();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if(!seenRgb || !seenDepth) throw std::runtime_error("RGB or depth did not start within 15 seconds");
    const auto started = Clock::now();
    const auto measureStart = started + std::chrono::duration<double>(warmup);
    const auto measureEnd = measureStart + std::chrono::duration<double>(duration);
    std::vector<Frame> rgbFrames, depthFrames;
    std::vector<double> validFractions;
    std::atomic<bool> done{false};
    std::uint64_t imuReports = 0;
    std::exception_ptr readerError;
    std::mutex errorMutex;
    auto read = [&](auto queue, std::vector<Frame>& frames, bool depth) {
        try {
            double lastQuality = -1;
            while(!done.load() && !interrupted) {
                auto packet = queue->template tryGet<dai::ImgFrame>();
                if(!packet) {std::this_thread::sleep_for(std::chrono::microseconds(100)); continue;}
                const auto now = Clock::now();
                if(now < measureStart || now >= measureEnd) continue;
                if(!depth && (packet->getWidth() != 1280 || packet->getHeight() != 800 ||
                              packet->getType() != dai::ImgFrame::Type::NV12))
                    throw std::runtime_error("RGB format changed from 1280x800 NV12");
                if(depth && (packet->getWidth() != width || packet->getHeight() != cropHeight ||
                             packet->getType() != dai::ImgFrame::Type::RAW16))
                    throw std::runtime_error("Depth output does not match the fixed cropped ROI");
                if(!packet->getTransformation().isValid()) throw std::runtime_error("Frame transformation metadata missing");
                const auto capture = packet->getTimestamp(dai::CameraExposureOffset::MIDDLE);
                const auto elapsed = std::chrono::duration<double>(now - measureStart).count();
                frames.push_back({elapsed, std::chrono::duration<double, std::milli>(now - capture).count(),
                                  packet->getSequenceNum(), std::chrono::duration_cast<std::chrono::nanoseconds>(capture.time_since_epoch()).count(),
                                  packet->getWidth(), packet->getHeight()});
                if(depth && elapsed - lastQuality >= 1.) {
                    lastQuality = elapsed;
                    const auto& bytes = packet->getData();
                    const auto stride = packet->getStride() ? packet->getStride() : width * 2;
                    if(bytes.size() < stride * cropHeight) throw std::runtime_error("Truncated depth frame");
                    std::size_t valid = 0;
                    for(unsigned y = 0; y < cropHeight; ++y) for(unsigned x = 0; x < width; ++x) {
                        std::uint16_t value;
                        std::memcpy(&value, bytes.data() + y * stride + x * 2, 2);
                        valid += value >= 100 && value <= 3000;
                    }
                    validFractions.push_back(double(valid) / (width * cropHeight));
                }
            }
        } catch(...) {
            std::lock_guard<std::mutex> lock(errorMutex);
            readerError = std::current_exception();
            done.store(true);
        }
    };
    std::thread rgbThread(read, rgbQueue, std::ref(rgbFrames), false);
    std::thread depthThread;
    if(depthQueue) depthThread = std::thread(read, depthQueue, std::ref(depthFrames), true);
    try {
        while(!done.load() && !interrupted && Clock::now() < measureEnd) {
            auto packet = imuQueue->tryGet<dai::IMUData>();
            if(packet && Clock::now() >= measureStart)
                imuReports += packet->packets.size();
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    } catch(...) {
        std::lock_guard<std::mutex> lock(errorMutex);
        readerError = std::current_exception();
    }
    done.store(true);
    rgbThread.join();
    if(depthThread.joinable()) depthThread.join();
    if(readerError) std::rethrow_exception(readerError);
    const double measured = std::clamp(std::chrono::duration<double>(Clock::now() - measureStart).count(), 0., duration);
    saveFrames(directory + "/rgb_frames.csv", rgbFrames);
    saveFrames(directory + "/depth_frames.csv", depthFrames);
    Json result = {{"config", config}, {"device_id", device->getDeviceId()},
                   {"usb_speed", int(device->getUsbSpeed())}, {"duration_sec", measured},
                   {"status", interrupted ? "interrupted" : "ok"},
                   {"rgb", frameStats(rgbFrames, std::max(.001, measured))},
                   {"depth", frameStats(depthFrames, std::max(.001, measured))},
                   {"imu_reports_hz", imuReports / std::max(.001, measured)},
                   {"depth_valid_fraction_0_1_to_3m", distribution(validFractions)},
                   {"roi", {{"x", 0}, {"y", cropTop}, {"width", width}, {"height", cropHeight},
                            {"bottom_fraction", .65}, {"applied", "both inputs before StereoDepth"}}}};
    if(rgbFrames.empty() || (stereoEnabled && depthFrames.empty())) result["status"] = "no_frames";
    pipeline.stop();
    pipeline.wait();
    return result;
}

int main(int argc, char** argv) {
    if(argc != 3) {std::cerr << "Usage: camera_probe config.json output-directory\n"; return 2;}
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
    Json result;
    int status = 0;
    try {
        std::ifstream input(argv[1]);
        if(!input) throw std::runtime_error("Cannot open config");
        Json config; input >> config;
        result = run(config, argv[2]);
        if(result.at("status") != "ok") status = 1;
    } catch(const std::exception& error) {
        result = {{"status", "error"}, {"error", error.what()}};
        std::cerr << error.what() << '\n';
        status = 1;
    }
    std::ofstream output(std::string(argv[2]) + "/result.json");
    if(!output) return 2;
    output << result.dump(2) << '\n';
    output.close();
    return output ? status : 2;
}
