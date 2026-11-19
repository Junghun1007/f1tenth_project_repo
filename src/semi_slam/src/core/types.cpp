#include "semi_slam/core/types.hpp"
#include <chrono>

namespace semi_slam {
double now_sec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace semi_slam
