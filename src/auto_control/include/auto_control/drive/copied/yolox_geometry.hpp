// Copied from src/multiclass_yolo/src/yolox_geometry.hpp (origin/1113ML)
#pragma once

#include <cstdint>

namespace auto_control::drive {

inline std::int64_t yolox_output_rows(int input_width, int input_height) {
  // The exported YOLOX head has one prediction per cell at each stride.
  const auto cells = [input_width, input_height](int stride) {
    return static_cast<std::int64_t>(input_width / stride) * (input_height / stride);
  };
  return cells(8) + cells(16) + cells(32);
}

}  // namespace auto_control::drive
