// 단계별 연산 시간: 최근 window 초의 평균 ms와 FPS(=1000/평균 ms, 기존 BEV HUD와 같은 기준)
#pragma once
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace cone_drive_test {

class StageStats {
public:
  explicit StageStats(double window_sec = 1.0) : window_(window_sec) {}
  void record(const std::string & stage, double start_sec, double end_sec);
  struct Row { std::string stage; double avg_ms, fps; };
  std::vector<Row> snapshot(double now) const;   // 기록 순서대로

private:
  struct Sample { double end, ms; };
  mutable std::mutex mutex_;
  double window_;
  std::vector<std::string> order_;
  mutable std::map<std::string, std::deque<Sample>> samples_;
};

}  // namespace cone_drive_test
