// IMU 영상 안정화 적용 판단과 적용률 집계.
// camera_driver는 1.25배 고정 확대 후 "보정 화면이 원본 안에 다 들어오는지" 검사했지만, 여기서는 확대 없이 BEV(화면 아래쪽)만
// 쓰므로 전체 화면 검사를 하지 않는다 (확대 없이 그 검사를 하면 0.1°만 돌려도 거부돼 안정화가 한 번도 적용되지 않음).
// 큰 보정은 ImuImageStabilizer의 maximum_correction_deg(3°) 제한이 막는다.
// 확대가 없으므로 fx/fy는 원본 값(bev_config 701.75 / 1.25 = 561.4)을 넣어야 한다 (camera_driver도 원본 K로 보정 homography를 만든다).
#pragma once
#include <mutex>
#include <string>
#include <opencv2/core.hpp>
#include "semi_slam/copied/imu_image_stabilizer.hpp"

namespace semi_slam {

struct StabilizationCamera { double fx, fy, cx, cy; int width, height; };

enum class StabilizationReason { Applied, Held, NotInitialized, NoImuMatch, OverLimit, Disabled };

struct StabilizationDecision {
  bool apply{false};
  cv::Matx33d source_to_stabilized{cv::Matx33d::eye()};
  StabilizationReason reason{StabilizationReason::NoImuMatch};
};

StabilizationDecision decide_stabilization(const StabilizationCamera & cam, const ImageStabilizationCorrection & c);

class StabilizationStats {   // 1초 단위 적용률 (스레드 안전). summary()가 집계를 비운다
public:
  void count(StabilizationReason r, double correction_deg);
  std::string summary();
  std::string last() const { std::lock_guard<std::mutex> l(m_); return last_; }

private:
  mutable std::mutex m_;
  int n_[6]{};
  double deg_sum_{0};
  std::string last_{"측정 전"};
};

}  // namespace semi_slam
