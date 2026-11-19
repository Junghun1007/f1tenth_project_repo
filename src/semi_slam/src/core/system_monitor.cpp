#include "semi_slam/core/system_monitor.hpp"
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>

namespace semi_slam {

SystemMonitor::SystemMonitor(std::string proc, std::string sys) : proc_(std::move(proc)), sys_(std::move(sys)) {
  const long t = sysconf(_SC_CLK_TCK);
  if (t > 0) clk_tck_ = t;
  ncpu_ = std::max(1u, std::thread::hardware_concurrency());
}

SystemUsage SystemMonitor::sample(double now) {
  SystemUsage u;
  {
    std::ifstream in(proc_ + "/stat");
    std::string cpu;
    long long v, busy = 0, total = 0;
    in >> cpu;
    std::vector<long long> f;
    while (f.size() < 8 && in >> v) f.push_back(v);
    if (f.size() >= 4) {
      for (auto x : f) total += x;
      busy = total - f[3] - (f.size() > 4 ? f[4] : 0);   // idle + iowait 제외
      if (prev_total_ >= 0 && total > prev_total_) u.cpu_percent = 100.0 * double(busy - prev_busy_) / double(total - prev_total_);
      prev_busy_ = busy;
      prev_total_ = total;
    }
  }
  {
    std::ifstream in(proc_ + "/self/stat");
    std::string line;
    std::getline(in, line);
    const auto rp = line.rfind(')');
    if (rp != std::string::npos) {
      std::istringstream ss(line.substr(rp + 2));
      std::string tok;
      long long utime = 0, stime = 0;
      for (int field = 3; ss >> tok; ++field) {   // 3번째 필드(state)부터
        if (field == 14) utime = std::stoll(tok);
        if (field == 15) { stime = std::stoll(tok); break; }
      }
      const long long ticks = utime + stime;
      if (prev_proc_ >= 0 && now > prev_t_)
        u.process_cpu_percent = 100.0 * double(ticks - prev_proc_) / double(clk_tck_) / (now - prev_t_) / ncpu_;
      prev_proc_ = ticks;
    }
  }
  {
    std::ifstream in(proc_ + "/meminfo");
    std::string key;
    long long val, total = 0, avail = -1;
    std::string unit;
    while (in >> key >> val) {
      std::getline(in, unit);
      if (key == "MemTotal:") total = val;
      if (key == "MemAvailable:") avail = val;
    }
    if (total > 0 && avail >= 0) u.mem_percent = 100.0 * double(total - avail) / double(total);
  }
  for (const char * p : {"/devices/platform/gpu.0/load", "/devices/gpu.0/load", "/devices/platform/17000000.gpu/load"}) {
    std::ifstream in(sys_ + p);
    double load;
    if (in >> load) { u.gpu_percent = load / 10.0; break; }
  }
  prev_t_ = now;
  return u;
}

}  // namespace semi_slam
