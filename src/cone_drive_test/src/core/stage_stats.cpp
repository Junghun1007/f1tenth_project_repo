#include "cone_drive_test/core/stage_stats.hpp"
#include <algorithm>

namespace cone_drive_test {

void StageStats::record(const std::string & stage, double start, double end) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & q = samples_[stage];
  if (std::find(order_.begin(), order_.end(), stage) == order_.end()) order_.push_back(stage);
  q.push_back({end, (end - start) * 1000.0});
  while (q.size() > 1 && q.front().end < end - window_) q.pop_front();
  if (q.size() > 4096) q.pop_front();
}

std::vector<StageStats::Row> StageStats::snapshot(double now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Row> rows;
  for (const auto & name : order_) {
    auto & q = samples_[name];
    while (!q.empty() && q.front().end < now - window_) q.pop_front();
    if (q.empty()) continue;
    double sum = 0;
    for (const auto & s : q) sum += s.ms;
    const double avg = sum / double(q.size());
    rows.push_back({name, avg, avg > 1e-9 ? 1000.0 / avg : 0.0});
  }
  return rows;
}

}  // namespace cone_drive_test
