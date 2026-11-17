#include "cone_drive_test/core/types.hpp"
#include <chrono>

namespace cone_drive_test {
double now_sec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace cone_drive_test
