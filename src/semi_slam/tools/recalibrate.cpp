// 저장한 주행 기록으로 보정만 다시 계산 (Jetson·PC 모두, ROS 불필요)
//   ros2 run semi_slam semi_slam_recalibrate <기록 폴더> [--apply] [--map <csv>] [--data <data_dir>]
// --apply: 지금 보정 값에 누적 반영해 저장 (이력에도 남김). 없으면 결과만 출력
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "semi_slam/core/run_store.hpp"

int main(int argc, char ** argv) {
  using namespace semi_slam;
  if (argc < 2) { std::fprintf(stderr, "usage: %s <run_dir> [--apply] [--map csv] [--data dir]\n", argv[0]); return 2; }
  std::string run_dir = argv[1], map_path, data_dir;
  bool apply = false;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--apply") apply = true;
    else if (a == "--map" && i + 1 < argc) map_path = argv[++i];
    else if (a == "--data" && i + 1 < argc) data_dir = argv[++i];
  }
  if (map_path.empty()) map_path = ament_index_cpp::get_package_share_directory("semi_slam") + "/maps/track_4x4.csv";
  if (data_dir.empty()) data_dir = std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/semi_slam_data";
  const auto map = TrackMap::load_csv(map_path);
  const auto run = load_run(run_dir);
  CalibStore store(data_dir);
  OdomCalib start = store.load();
  if (start.version < OdomCalib::kVersion) start = OdomCalib{};
  std::printf("기록 %s: 속도 %.2f m/s, 바퀴 %zu, 차선 프레임 %zu, 방향 %s\n", run_dir.c_str(), run.target_speed, run.crossing_t.size(),
              run.lanes.size(), run.cw ? "시계" : "반시계");
  const auto r = calibrate(run, start, map);
  const auto & e = r.est;
  std::printf("이번 주행 값: 자이로 ×%.4f (원값 한 바퀴 %.2f°), 거리 ×%.4f (원값 한 바퀴 %.3f m, 기준 %.3f m)\n", e.gyro_scale,
              e.raw_turn_deg_per_lap, e.wheel_scale, e.raw_dist_per_lap, e.ref_length);
  if (std::isfinite(e.bev_yaw_residual_deg))
    std::printf("주행 검산: 직선 %d개, BEV 차선 각 %+.2f° − 실제 비스듬함 %+.2f° = BEV가 더 기울어 보임 %+.2f°\n", e.straights,
                e.straight_angle_deg, e.straight_slip_deg, e.bev_yaw_residual_deg);
  std::printf("%s\n보정 값 %s\n", r.message.c_str(), r.calib.to_json().c_str());
  for (const auto & m : r.after.laps)
    std::printf("  바퀴 %d: 거리 %.3f m (S 도착 %+.1f cm), 회전 %.1f°, 자유 적분 S 오차 %+.1f cm %+.1f°\n", m.lap, m.dist_m, m.s_err_m * 100,
                m.turn_deg, m.along_err_m * 100, m.heading_err_deg);
  write_calibration_result(run_dir, r);
  if (apply && r.applied) {
    store.record(r, run_dir, start, run.target_speed, int(run.crossing_t.size()), true);
    store.save(r.calib);
    std::printf("적용함: %s/calibration.json (semi_slam을 다시 실행하면 반영)\n", data_dir.c_str());
  } else if (apply) {
    std::printf("반영할 값이 없어 저장하지 않음\n");
  }
  return 0;
}
