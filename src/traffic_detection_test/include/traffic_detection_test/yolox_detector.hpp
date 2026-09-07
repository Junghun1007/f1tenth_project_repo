#ifndef TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_
#define TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"

namespace traffic_detection_test
{

struct TrafficLightDetection
{
  cv::Rect2f box;
  float score;
};

struct YoloxStageTiming
{
  std::uint64_t preprocessing_nanoseconds{0U};
  std::uint64_t forward_nanoseconds{0U};
  std::uint64_t postprocessing_nanoseconds{0U};
};

struct YoloxDetectionResult
{
  std::vector<TrafficLightDetection> detections;
  YoloxStageTiming timing;
};

class YoloxDetector
{
public:
  YoloxDetector(
    const std::string & model_path,
    const std::string & inference_backend,
    int input_width,
    int input_height,
    float score_threshold,
    float nms_threshold);

  YoloxDetectionResult detect(const cv::Mat & bgr_image);
  void draw(
    cv::Mat & bgr_image,
    const std::vector<TrafficLightDetection> & detections) const;
  const std::string & backend_name() const noexcept;

private:
  static float intersection_over_union(
    const cv::Rect2f & first, const cv::Rect2f & second);
  static std::vector<std::size_t> nms(
    const std::vector<TrafficLightDetection> & detections,
    float threshold);

  cv::dnn::Net network_;
  int input_width_;
  int input_height_;
  float score_threshold_;
  float nms_threshold_;
  std::string backend_name_;
};

}  // namespace traffic_detection_test

#endif  // TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_
