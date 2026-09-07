#include "traffic_detection_test/yolox_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace traffic_detection_test
{

YoloxDetector::YoloxDetector(
  const std::string & model_path,
  const int input_width,
  const int input_height,
  const float score_threshold,
  const float nms_threshold)
: input_width_(input_width),
  input_height_(input_height),
  score_threshold_(score_threshold),
  nms_threshold_(nms_threshold)
{
  if (input_width_ <= 0 || input_height_ <= 0) {
    throw std::invalid_argument("model input dimensions must be positive");
  }
  if (
    !std::isfinite(score_threshold_) || score_threshold_ < 0.0F ||
    score_threshold_ > 1.0F)
  {
    throw std::invalid_argument("score_threshold must be in [0, 1]");
  }
  if (
    !std::isfinite(nms_threshold_) || nms_threshold_ < 0.0F ||
    nms_threshold_ > 1.0F)
  {
    throw std::invalid_argument("nms_threshold must be in [0, 1]");
  }

  network_ = cv::dnn::readNetFromONNX(model_path);
  if (network_.empty()) {
    throw std::runtime_error("OpenCV could not load the ONNX model");
  }

  // Keep the reference implementation in FP32. Quantized and TensorRT
  // variants are intentionally outside this preview-only package.
  network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

std::vector<TrafficLightDetection> YoloxDetector::detect(
  const cv::Mat & bgr_image)
{
  if (bgr_image.empty()) {
    throw std::invalid_argument("cannot run detection on an empty image");
  }
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("YOLOX input must be an 8-bit BGR image");
  }

  const float ratio = std::min(
    static_cast<float>(input_height_) /
    static_cast<float>(bgr_image.rows),
    static_cast<float>(input_width_) /
    static_cast<float>(bgr_image.cols));
  const int resized_width = std::max(
    1, static_cast<int>(static_cast<float>(bgr_image.cols) * ratio));
  const int resized_height = std::max(
    1, static_cast<int>(static_cast<float>(bgr_image.rows) * ratio));

  cv::Mat resized;
  cv::resize(
    bgr_image, resized, cv::Size(resized_width, resized_height),
    0.0, 0.0, cv::INTER_LINEAR);

  // Match training/infer_onnx.py exactly: BGR, 0..255 FP32, top-left
  // placement, and 114-valued padding. A 640x400 frame therefore receives
  // 240 rows of bottom padding without being stretched.
  cv::Mat padded(
    input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(0, 0, resized.cols, resized.rows)));

  cv::Mat blob = cv::dnn::blobFromImage(
    padded,
    1.0,
    cv::Size(input_width_, input_height_),
    cv::Scalar(),
    false,
    false,
    CV_32F);
  network_.setInput(blob);
  cv::Mat output = network_.forward();

  int row_count = 0;
  int column_count = 0;
  if (output.dims == 3 && output.size[0] == 1) {
    row_count = output.size[1];
    column_count = output.size[2];
  } else if (output.dims == 2) {
    row_count = output.rows;
    column_count = output.cols;
  } else {
    throw std::runtime_error(
            "unexpected YOLOX output rank; expected [1, N, 6]");
  }
  if (column_count != 6 || output.type() != CV_32F) {
    throw std::runtime_error(
            "unexpected YOLOX output; expected FP32 [1, N, 6]");
  }
  if (!output.isContinuous()) {
    output = output.clone();
  }

  const float * rows = output.ptr<float>();
  std::vector<TrafficLightDetection> candidates;
  candidates.reserve(static_cast<std::size_t>(row_count));
  for (int index = 0; index < row_count; ++index) {
    const float * row = rows + static_cast<std::size_t>(index) * 6U;
    const float score = row[4] * row[5];
    if (!std::isfinite(score) || score < score_threshold_) {
      continue;
    }

    float left = (row[0] - row[2] * 0.5F) / ratio;
    float top = (row[1] - row[3] * 0.5F) / ratio;
    float right = (row[0] + row[2] * 0.5F) / ratio;
    float bottom = (row[1] + row[3] * 0.5F) / ratio;
    left = std::clamp(left, 0.0F, static_cast<float>(bgr_image.cols - 1));
    top = std::clamp(top, 0.0F, static_cast<float>(bgr_image.rows - 1));
    right = std::clamp(
      right, 0.0F, static_cast<float>(bgr_image.cols - 1));
    bottom = std::clamp(
      bottom, 0.0F, static_cast<float>(bgr_image.rows - 1));
    if (right <= left || bottom <= top) {
      continue;
    }

    candidates.push_back(
      TrafficLightDetection{
        cv::Rect2f(left, top, right - left, bottom - top), score});
  }

  const auto kept_indices = nms(candidates, nms_threshold_);
  std::vector<TrafficLightDetection> detections;
  detections.reserve(kept_indices.size());
  for (const auto index : kept_indices) {
    detections.push_back(candidates[index]);
  }
  return detections;
}

void YoloxDetector::draw(
  cv::Mat & bgr_image,
  const std::vector<TrafficLightDetection> & detections) const
{
  const cv::Scalar box_color(255, 80, 180);
  for (const auto & detection : detections) {
    const int left = static_cast<int>(std::lround(detection.box.x));
    const int top = static_cast<int>(std::lround(detection.box.y));
    const int right = static_cast<int>(
      std::lround(detection.box.x + detection.box.width));
    const int bottom = static_cast<int>(
      std::lround(detection.box.y + detection.box.height));
    cv::rectangle(
      bgr_image, cv::Point(left, top), cv::Point(right, bottom),
      box_color, 2, cv::LINE_AA);

    const std::string label = cv::format(
      "traffic_light %.2f", detection.score);
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
    const int label_y = std::max(text_size.height + 3, top);
    cv::rectangle(
      bgr_image,
      cv::Point(left, label_y - text_size.height - 3),
      cv::Point(left + text_size.width + 4, label_y + baseline),
      box_color,
      cv::FILLED);
    cv::putText(
      bgr_image,
      label,
      cv::Point(left + 2, label_y - 2),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(255, 255, 255),
      1,
      cv::LINE_AA);
  }
}

float YoloxDetector::intersection_over_union(
  const cv::Rect2f & first, const cv::Rect2f & second)
{
  const float left = std::max(first.x, second.x);
  const float top = std::max(first.y, second.y);
  const float right = std::min(
    first.x + first.width, second.x + second.width);
  const float bottom = std::min(
    first.y + first.height, second.y + second.height);
  const float intersection =
    std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float union_area = first.area() + second.area() - intersection;
  return union_area > 0.0F ? intersection / union_area : 0.0F;
}

std::vector<std::size_t> YoloxDetector::nms(
  const std::vector<TrafficLightDetection> & detections,
  const float threshold)
{
  std::vector<std::size_t> order(detections.size());
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(
    order.begin(), order.end(),
    [&detections](const std::size_t first, const std::size_t second) {
      return detections[first].score > detections[second].score;
    });

  std::vector<std::size_t> kept;
  while (!order.empty()) {
    const std::size_t current = order.front();
    kept.push_back(current);
    order.erase(order.begin());
    order.erase(
      std::remove_if(
        order.begin(), order.end(),
        [&detections, current, threshold](const std::size_t candidate) {
          return intersection_over_union(
            detections[current].box, detections[candidate].box) > threshold;
        }),
      order.end());
  }
  return kept;
}

}  // namespace traffic_detection_test
