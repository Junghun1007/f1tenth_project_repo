#include "semi_slam/core/stage_stats.hpp"
#include <algorithm>

namespace semi_slam {

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
    rows.push_back({name, avg, double(q.size()) / window_});   // 실제 처리 횟수/초 (1000/ms가 아님: 지연 단계에선 의미 없음)
  }
  return rows;
}

}  // namespace semi_slam
