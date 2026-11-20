#include "check.hpp"
#include "auto_control/drive/core/lane_post.hpp"
#include <opencv2/imgproc.hpp>

using namespace auto_control::drive;

// 120×300 BEV 라벨: 1=왼쪽, 2=오른쪽. 앞차축 기준 y = ±0.33 m 직선 차선
static cv::Mat straight_labels(double y_left = 0.33, double y_right = -0.33) {
  cv::Mat l(300, 120, CV_8UC1, cv::Scalar(0));
  const int ul = int(std::lround(0.6 / 0.01 - y_left / 0.01 - 0.5)), ur = int(std::lround(0.6 / 0.01 - y_right / 0.01 - 0.5));
  cv::line(l, {ul, 0}, {ul, 299}, cv::Scalar(1), 3);
  cv::line(l, {ur, 0}, {ur, 299}, cv::Scalar(2), 3);
  return l;
}

TEST(straight_lanes_give_center_at_zero) {
  LanePostprocessor post(LanePostConfig{});
  const auto lane = post.process(straight_labels(), 1.0, false);
  CHECK(lane.valid);
  CHECK(lane.path.size() > 20);
  for (const auto & p : lane.path) CHECK_NEAR(p.y, 0.0, 0.02);
  CHECK(lane.path.front().x < lane.path.back().x);   // 가까운 점부터
  CHECK(!lane.left.empty() && !lane.right.empty());
  CHECK_NEAR(lane.left.front().y, 0.33, 0.02);
  CHECK_NEAR(lane.right.front().y, -0.33, 0.02);
  CHECK(lane.center.empty());                         // 콘 없음 → 기준선 추가 계산 안 함
  CHECK_NEAR(lane.capture_time, 1.0, 1e-12);
}

TEST(reference_center_computed_only_when_requested) {
  LanePostprocessor post(LanePostConfig{});
  const auto lane = post.process(straight_labels(), 1.0, true);
  CHECK(lane.center.size() > 20);
}

TEST(empty_labels_invalid) {
  LanePostprocessor post(LanePostConfig{});
  const cv::Mat empty(300, 120, CV_8UC1, cv::Scalar(0));
  CHECK(!post.process(empty, 0.0, false).valid);
}

TEST(shifted_lane_center_offset) {
  LanePostprocessor post(LanePostConfig{});
  const auto lane = post.process(straight_labels(0.43, -0.23), 0.0, false);   // 차선 중심이 왼쪽 0.10
  CHECK(lane.valid);
  CHECK_NEAR(lane.path[lane.path.size() / 2].y, 0.10, 0.02);
}
