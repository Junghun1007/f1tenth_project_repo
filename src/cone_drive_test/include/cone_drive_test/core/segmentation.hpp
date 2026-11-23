#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>

namespace cone_drive_test {

constexpr int seg_width = 640, seg_height = 320, seg_anchors = 4200, seg_channels = 32;
constexpr int proto_width = 160, proto_height = 80;
constexpr int seg_prediction_values = (8 + seg_channels) * seg_anchors;
constexpr int seg_output_values = seg_prediction_values + seg_channels * proto_width * proto_height;

struct SegTransform {
  cv::Rect roi;
  cv::Rect content;
  cv::Rect2f source_box(const cv::Rect2f & box) const;
  cv::Point model_pixel(double rgb_x, double rgb_y) const;
};

SegTransform seg_transform(const cv::Rect & roi);

struct SegDetection {
  cv::Rect2f box, model_box;
  float score{};
  int class_id{};
  std::array<float, seg_channels> coefficients{};
};

std::vector<SegDetection> decode_seg(const std::uint16_t * packed, const SegTransform & transform,
                                     const std::array<float, 4> & thresholds, float nms_iou);
bool mask_contains(const std::uint16_t * prototypes, const SegDetection & det, cv::Point model_pixel);

}  // namespace cone_drive_test
