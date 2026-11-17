// 오도메트리: 속도·요레이트 적분 (뒷차축 기준 적분, 앞차축 자세 출력) + 최근 2 s 이력 보간
#pragma once
#include <deque>
#include <optional>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

class Odometry {
public:
  explicit Odometry(double wheelbase, double history_sec = 2.0) : L_(wheelbase), history_(history_sec) {}
  void reset(const Pose2 & front_pose, double t = 0.0);
  void update(double t, double speed, double yaw_rate);   // t는 단조 증가 (s)
  Pose2 pose() const;
  std::optional<Pose2> pose_at(double t) const;          // 이력 범위 밖이면 nullopt
  double last_time() const { return hist_.empty() ? 0.0 : hist_.back().t; }
  double distance() const { return dist_; }                 // 누적 주행 거리 (|v| 적분, 자이로 무관)
  std::optional<double> distance_at(double t) const;        // 이력 범위 밖이면 nullopt

private:
  struct Sample { double t; Pose2 front; double dist; };
  double L_, history_;
  double rx_{0}, ry_{0}, psi_{0}, dist_{0};
  std::deque<Sample> hist_;
};

}  // namespace cone_drive_test
