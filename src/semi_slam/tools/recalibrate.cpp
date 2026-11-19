// 저장한 주행 기록으로 보정만 다시 계산 (Jetson·PC 모두, ROS 불필요)
//   ros2 run semi_slam semi_slam_recalibrate <기록 폴더> [--apply] [--map <csv>] [--data <data_dir>]
// --apply: 오차가 줄었으면 지금 보정 값으로 저장 (이력에도 남김). 없으면 결과만 출력
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
  const OdomCalib start = store.load();
  std::printf("기록 %s: 속도 %.2f m/s, 바퀴 %zu, 차선 프레임 %zu\n", run_dir.c_str(), run.target_speed, run.crossing_t.size(), run.lanes.size());
  const auto r = calibrate(run, start, map);
  std::printf("정합 RMS %.1f → %.1f cm, 정확도 %.1f%% → %.1f%% (%s, 계산 %d회)\n", r.before.rms_m * 100, r.after.rms_m * 100,
              r.before.accuracy * 100, r.after.accuracy * 100, r.message.c_str(), r.evaluations);
  std::printf("보정 값 %s\n", r.calib.to_json().c_str());
  for (const auto & m : r.after.laps)
    std::printf("  바퀴 %d: 통과 위치 %+.1f cm, 방향 %+.1f°, 길이 %+.1f%%, RMS %.1f cm\n", m.lap, m.along_err_m * 100, m.heading_err_deg,
                m.length_err * 100, m.rms_m * 100);
  write_calibration_result(run_dir, r);
  if (apply && r.improved) {
    store.record(r, run_dir, start, run.target_speed, int(run.crossing_t.size()), true);
    store.save(r.calib);
    std::printf("적용함: %s/calibration.json (semi_slam을 다시 실행하면 반영)\n", data_dir.c_str());
  } else if (apply) {
    std::printf("오차가 줄지 않아 적용하지 않음\n");
  }
  return 0;
}
