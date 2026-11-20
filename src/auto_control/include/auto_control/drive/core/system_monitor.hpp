// Copied from src/cone_drive_test/include/cone_drive_test/core/system_monitor.hpp (1113ML) — auto_control로 옮김
// CPU·메모리·GPU 사용률 (Linux /proc, Jetson GPU sysfs load 0~1000)
#pragma once
#include <optional>
#include <string>

namespace auto_control::drive {

struct SystemUsage {
  double cpu_percent{0}, process_cpu_percent{0}, mem_percent{0};
  std::optional<double> gpu_percent;
};

class SystemMonitor {
public:
  explicit SystemMonitor(std::string proc_root = "/proc", std::string sys_root = "/sys");
  SystemUsage sample(double now_sec);   // 이전 호출과의 차이로 CPU % 계산

private:
  std::string proc_, sys_;
  long long prev_busy_{-1}, prev_total_{-1}, prev_proc_{-1};
  double prev_t_{0};
  long clk_tck_{100};
  int ncpu_{1};
};

}  // namespace auto_control::drive
