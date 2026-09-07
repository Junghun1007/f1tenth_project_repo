#ifndef TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_
#define TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_

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

class YoloxDetector
{
public:
  YoloxDetector(
    const std::string & model_path,
    int input_width,
    int input_height,
    float score_threshold,
    float nms_threshold);

  std::vector<TrafficLightDetection> detect(const cv::Mat & bgr_image);
  void draw(
    cv::Mat & bgr_image,
    const std::vector<TrafficLightDetection> & detections) const;

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
};

}  // namespace traffic_detection_test

#endif  // TRAFFIC_DETECTION_TEST__YOLOX_DETECTOR_HPP_
