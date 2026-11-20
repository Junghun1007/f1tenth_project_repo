// Copied from src/cone_drive_test/include/cone_drive_test/core/path_lock.hpp (1113ML) — auto_control로 옮김
// 기억 방식 2: "보이는 콘으로만 계획 + 화면을 벗어난 콘 옆은 경로 고정".
// 콘이 카메라 사각지대(차선 기준 s < visible_s_min)로 들어가면, 그 콘이 보일 때 세운 직전 경로를
// 뒷범퍼가 콘을 지날 때까지(앞차축 기준 s + 앞차축→뒷범퍼) 고정한다. 기억 위치로 다시 계획하지 않으므로
// 사각지대 콘 위치 오차 때문에 경로가 튀지 않는다. 좌표는 차선 기준 (s, l), 이동은 바퀴 거리로만 당긴다.
#pragma once
#include <vector>
#include "auto_control/drive/core/frenet_planner.hpp"

namespace auto_control::drive {

class PathLock {
public:
  bool active() const { return end_s_ > 0 && pin_.s.size() >= 2; }
  double remaining() const { return active() ? end_s_ : 0.0; }
  void advance(double ds) {   // 차가 ds 전진
    if (!(ds > 0)) return;
    for (auto & s : pin_.s) s -= ds;
    for (auto & q : last_plan_) q.x -= ds;
    end_s_ -= ds;
    pin_.end_s = end_s_;
    if (end_s_ <= 0) { pin_.s.clear(); pin_.l.clear(); }
  }
  // blind_end: 사각지대 콘들이 요구하는 고정 끝 (없으면 ≤0). 새 콘이 사각지대로 들어와 끝이 늘어나면 직전 경로로 다시 고정
  void refresh(double blind_end) {
    if (blind_end <= end_s_ + 0.05 || last_plan_.size() < 2) return;
    pin_.s.clear();
    pin_.l.clear();
    for (const auto & q : last_plan_) {
      if (!pin_.s.empty() && q.x <= pin_.s.back()) continue;   // s 오름차순만
      pin_.s.push_back(q.x);
      pin_.l.push_back(q.y);
      if (q.x > blind_end + 0.2) break;
    }
    end_s_ = blind_end;
    pin_.end_s = end_s_;
  }
  void set_last_plan(std::vector<Vec2> sl) { last_plan_ = std::move(sl); }   // 이번 계획 경로 (s, l)
  const LanePin * pin() const { return active() ? &pin_ : nullptr; }
  void clear() { pin_.s.clear(); pin_.l.clear(); last_plan_.clear(); end_s_ = -1; }

private:
  LanePin pin_;
  std::vector<Vec2> last_plan_;
  double end_s_{-1};
};

}  // namespace auto_control::drive
