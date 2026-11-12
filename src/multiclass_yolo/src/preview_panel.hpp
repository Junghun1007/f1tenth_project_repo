#pragma once

#include "multiclass_yolo/yolox_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace multiclass_yolo {

class PreviewPanel {
 public:
  static constexpr int width = 324;
  static constexpr int min_height = 440;
  static constexpr int fps_field = 0;
  static constexpr int roi_field = 1;
  static constexpr int first_threshold_field = 2;

  struct Edit {
    int field;
    std::optional<double> value;
  };

  void click(int x, int y) {
    const cv::Point position{x, y};
    if (page_ > 0 && previous_page_.contains(position)) {
      --page_;
      dirty = true;
      return;
    }
    if (page_ + 1 < page_count_ && next_page_.contains(position)) {
      ++page_;
      dirty = true;
      return;
    }
    if (active_ && apply_rect(*active_).contains(position)) {
      submit_ = true;
      return;
    }
    for (int field = fps_field; field < first_threshold_field + 4; ++field) {
      if (input_rect(field).contains(position)) {
        if (active_ != field) begin(field);
        return;
      }
    }
  }

  bool handle_key(int key) {
    if (key == 'f' || key == 'F') {
      begin(fps_field);
      return true;
    }
    if (!active_) return false;
    if (key == 27) {
      finish({});
      return true;
    }
    if (key == 10 || key == 13) {
      submit_ = true;
      return true;
    }
    if (key == 8 || key == 127) {
      if (!value_.empty()) value_.pop_back();
    } else if (value_.size() < 10 &&
      ((key >= '0' && key <= '9') || (key == '.' && value_.find('.') == std::string::npos)))
    {
      value_ += static_cast<char>(key);
    } else {
      return false;
    }
    message_.clear();
    dirty = true;
    return true;
  }

  std::optional<Edit> take_submission() {
    if (!submit_ || !active_) return std::nullopt;
    submit_ = false;
    std::istringstream stream(value_);
    stream.imbue(std::locale::classic());
    double value;
    if (stream >> value && stream.eof()) return Edit{*active_, value};
    return Edit{*active_, std::nullopt};
  }

  void finish(const std::string & message) {
    message_ = message;
    if (message_.empty()) active_.reset();
    submit_ = false;
    dirty = true;
  }

  void draw(
    cv::Mat & canvas, double fps, int roi_percent,
    const std::array<const char *, 4> & names,
    const std::array<float, 4> & thresholds,
    const std::array<float, 4> & detection_thresholds,
    const std::vector<Detection> & detections,
    const std::string & camera_status)
  {
    cv::Mat panel = canvas.colRange(0, width);
    panel.setTo(cv::Scalar(28, 26, 24));
    draw_input(panel, fps_field, "Camera FPS", fps);
    draw_input(panel, roi_field, "Bottom ROI %", roi_percent);
    text(panel, "Detection thresholds", {14, 100}, .58);
    for (std::size_t i = 0; i < names.size(); ++i)
      draw_input(panel, first_threshold_field + static_cast<int>(i), names[i], thresholds[i]);

    if (!message_.empty()) {
      text(panel, message_, {14, 262}, .48, {100, 180, 255});
    } else if (!camera_status.empty()) {
      text(panel, camera_status, {14, 262}, .48);
    } else if (active_) {
      text(panel, "Enter to apply | Esc to cancel", {14, 262}, .48);
    }

    cv::line(panel, {14, 278}, {width - 14, 278}, {72, 68, 64}, 1);
    text(panel, cv::format("Detections (%zu)", detections.size()), {14, 306}, .62);
    text(panel, "Class", {14, 333});
    text(panel, "Score", {160, 333});
    text(panel, "Threshold", {230, 333});
    constexpr int first_row_y = 360;
    constexpr int row_height = 24;
    const auto capacity = static_cast<std::size_t>((panel.rows - 44 - first_row_y) / row_height + 1);
    page_count_ = std::max<std::size_t>(1, (detections.size() + capacity - 1) / capacity);
    page_ = std::min(page_, page_count_ - 1);
    if (detections.empty()) {
      text(panel, "No detections", {14, first_row_y}, .5, {175, 175, 175});
      return;
    }

    const auto first = page_ * capacity;
    const auto end = std::min(first + capacity, detections.size());
    int y = first_row_y;
    for (std::size_t i = first; i < end; ++i, y += row_height) {
      const auto & detection = detections[i];
      text(panel, names[detection.class_id], {14, y});
      text(panel, cv::format("%.3f", detection.score), {160, y});
      text(panel, cv::format("%.6g", detection_thresholds[detection.class_id]), {230, y});
    }
    if (page_count_ > 1) {
      const int top = panel.rows - 35;
      previous_page_ = {14, top, 64, 26};
      next_page_ = {width - 78, top, 64, 26};
      if (page_ > 0) {
        cv::rectangle(panel, previous_page_, cv::Scalar(65, 65, 65), cv::FILLED);
        text(panel, "Prev", {previous_page_.x + 12, top + 17});
      }
      if (page_ + 1 < page_count_) {
        cv::rectangle(panel, next_page_, cv::Scalar(65, 65, 65), cv::FILLED);
        text(panel, "Next", {next_page_.x + 12, top + 17});
      }
      text(panel, cv::format("%zu / %zu", page_ + 1, page_count_), {126, top + 17});
    }
  }

  bool dirty{true};

 private:
  void begin(int field) {
    active_ = field;
    value_.clear();
    message_.clear();
    submit_ = false;
    dirty = true;
  }

  static int row_top(int field) {
    return field < first_threshold_field ? 12 + field * 36 :
           109 + (field - first_threshold_field) * 34;
  }

  static cv::Rect input_rect(int field) {
    return {145, row_top(field), 111, 28};
  }

  static cv::Rect apply_rect(int field) {
    return {262, row_top(field), 50, 28};
  }

  static void text(
    cv::Mat & panel, const std::string & value, cv::Point position, double scale = .5,
    cv::Scalar color = {235, 235, 235})
  {
    cv::putText(panel, value, position, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
  }

  void draw_input(cv::Mat & panel, int field, const char * label, double value) const {
    const auto input = input_rect(field);
    const bool editing = active_ == field;
    const int y = input.y + 19;
    text(panel, label, {14, y});
    cv::rectangle(panel, input, editing ? cv::Scalar(100, 210, 255) : cv::Scalar(120, 120, 120), 1);
    text(panel, editing ? value_ + "_" : cv::format("%.6g", value), {input.x + 6, y});
    if (editing) {
      const auto apply = apply_rect(field);
      cv::rectangle(panel, apply, cv::Scalar(65, 65, 65), cv::FILLED);
      text(panel, "Apply", {apply.x + 6, y}, .44);
    }
  }

  cv::Rect previous_page_, next_page_;
  std::size_t page_{0}, page_count_{1};
  std::optional<int> active_;
  std::string value_, message_;
  bool submit_{false};
};

}  // namespace multiclass_yolo
