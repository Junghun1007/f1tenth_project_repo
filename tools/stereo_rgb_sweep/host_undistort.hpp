#pragma once
#include <depthai/depthai.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

// Latest-frame CPU reference for moving RGB lens correction to Jetson.
// Queue ownership keeps acquisition independent of host processing duration.
class HostUndistort {
    std::mutex mutex_;
    std::condition_variable ready_;
    std::shared_ptr<dai::ImgFrame> latest_;
    bool stop_{false};
    std::chrono::steady_clock::time_point start_;
    double duration_;
    std::thread worker_;
    cv::Mat mapX_, mapY_;
    void process() {
        try {
            while(true) {
                std::shared_ptr<dai::ImgFrame> frame;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    ready_.wait(lock, [this] {return stop_ || latest_;});
                    if(stop_) break;
                    frame = std::move(latest_);
                }
                const auto began = std::chrono::steady_clock::now();
                if(mapX_.empty()) {
                    const auto& transform = frame->getTransformation();
                    auto intrinsic = transform.getIntrinsicMatrix();
                    cv::Mat k(3, 3, CV_64F);
                    for(int y = 0; y < 3; ++y)
                        for(int x = 0; x < 3; ++x) k.at<double>(y, x) = intrinsic[y][x];
                    const auto coefficients = transform.getDistortionCoefficients();
                    if(coefficients.empty()) throw std::runtime_error("Raw RGB distortion metadata missing");
                    cv::Mat d(coefficients, true);
                    d.convertTo(d, CV_64F);
                    const cv::Size size(frame->getWidth(), frame->getHeight());
                    if(transform.getDistortionModel() == dai::CameraModel::Fisheye) {
                        if(coefficients.size() < 4) throw std::runtime_error("Invalid fisheye distortion metadata");
                        cv::fisheye::initUndistortRectifyMap(k, d.rowRange(0, 4), cv::Mat::eye(3, 3, CV_64F), k,
                                                           size, CV_32FC1, mapX_, mapY_);
                    } else if(transform.getDistortionModel() == dai::CameraModel::Perspective) {
                        cv::initUndistortRectifyMap(k, d, cv::Mat::eye(3, 3, CV_64F), k, size, CV_32FC1, mapX_, mapY_);
                    } else throw std::runtime_error("Unsupported distortion model for host correction");
                }
                auto bgr = frame->getCvFrame();
                cv::Mat corrected;
                cv::remap(bgr, corrected, mapX_, mapY_, cv::INTER_LINEAR);
                const auto now = std::chrono::steady_clock::now();
                const double elapsed = std::chrono::duration<double>(now - start_).count();
                if(elapsed >= 0 && elapsed < duration_) {
                    completionSeconds.push_back(elapsed);
                    computeMs.push_back(std::chrono::duration<double, std::milli>(now - began).count());
                }
            }
        } catch(...) {error = std::current_exception();}
    }
public:
    std::vector<double> completionSeconds, computeMs;
    std::exception_ptr error;
    HostUndistort(std::chrono::steady_clock::time_point start, double duration)
        : start_(start), duration_(duration), worker_() {
        worker_ = std::thread(&HostUndistort::process, this);
    }
    ~HostUndistort() {close();}
    void submit(std::shared_ptr<dai::ImgFrame> frame) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_ = std::move(frame);
        }
        ready_.notify_one();
    }
    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        ready_.notify_all();
        if(worker_.joinable()) worker_.join();
    }
};
