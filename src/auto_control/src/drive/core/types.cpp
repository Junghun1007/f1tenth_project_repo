// Copied from src/cone_drive_test/src/core/types.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/types.hpp"
#include <chrono>

namespace auto_control::drive {
double now_sec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace auto_control::drive
