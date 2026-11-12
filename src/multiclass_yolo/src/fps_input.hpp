#pragma once

#include <opencv2/imgproc.hpp>

#include <locale>
#include <optional>
#include <sstream>
#include <string>

namespace multiclass_yolo {

class FpsInput {
 public:
  void click(int x, int y) {
    const cv::Point position{x, y};
    if (field_.contains(position) && !editing_) begin();
    if (apply_.contains(position) && editing_) submit = true;
  }

  bool handle_key(int key) {
    if (key == 'f' || key == 'F') {
      begin();
      return true;
    }
    if (!editing_) return false;
    if (key == 27) {
      finish();
      return true;
    }
    if (key == 10 || key == 13) {
      submit = true;
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
    message.clear();
    dirty = true;
    return true;
  }

  std::optional<double> value() const {
    std::istringstream stream(value_);
    stream.imbue(std::locale::classic());
    double fps;
    if (stream >> fps && stream.eof()) return fps;
    return std::nullopt;
  }

  void finish() {
    editing_ = false;
    message.clear();
    dirty = true;
  }

  void draw(cv::Mat & canvas, double fps, const std::string & camera_status) const {
    canvas.rowRange(0, 44).setTo(cv::Scalar(20, 20, 20));
    const cv::Scalar text(240, 240, 240);
    const auto label = [&](const std::string & value, cv::Point pos) {
      cv::putText(canvas, value, pos, cv::FONT_HERSHEY_SIMPLEX, .55, text, 1, cv::LINE_AA);
    };
    label("Camera FPS", {10, 28});
    cv::rectangle(canvas, field_, editing_ ? cv::Scalar(100, 210, 255) : text, 1);
    label(editing_ ? value_ + "_" : cv::format("%.6g", fps), {121, 28});
    cv::rectangle(canvas, apply_, cv::Scalar(65, 65, 65), cv::FILLED);
    label("Apply", {250, 28});
    if (editing_ && !message.empty()) {
      label(message, {325, 28});
    } else if (!camera_status.empty()) {
      label(camera_status, {325, 28});
    } else if (editing_) {
      label("Enter to apply", {325, 28});
    }
  }

  bool dirty{true};
  bool submit{false};
  std::string message;

 private:
  void begin() {
    editing_ = true;
    value_.clear();
    message.clear();
    dirty = true;
  }

  const cv::Rect field_{113, 8, 121, 28};
  const cv::Rect apply_{241, 8, 67, 28};
  bool editing_{false};
  std::string value_;
};

}  // namespace multiclass_yolo
