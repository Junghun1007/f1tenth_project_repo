// 주행 기록 폴더와 보정 값 저장소.
//  <data_dir>/runs/<날짜_시각_v속도>/  meta.json, speed.csv, gyro.csv, lanes.csv, crossings.csv, laps.csv,
//                                       corrections.csv, trajectory.csv, calibration.json(보정 결과)
//  <data_dir>/calibration.json          지금 쓰는 보정 값 (다음 실행에도 이어서)
//  <data_dir>/calibration_history.jsonl 보정 이력 (한 줄 = 한 번). 되돌리기는 직전 적용의 이전 값으로
//  <data_dir>/odometry_calibration.yaml cone_drive_test 등 다른 패키지가 읽기 쉬운 형식
#pragma once
#include <string>
#include <vector>
#include "semi_slam/core/slam.hpp"

namespace semi_slam {

std::string make_run_dir(const std::string & data_dir, double target_speed);   // 폴더 생성 후 경로
void write_run(const std::string & dir, const RunData & run, const OdomCalib & calib_used, const std::vector<LapMetrics> & laps,
               const std::vector<std::pair<int, double>> & corrections, const std::vector<Pose> & trajectory);
void write_calibration_result(const std::string & dir, const CalibResult & r);
RunData load_run(const std::string & dir);   // 기록 폴더 → RunData (PC에서 다시 보정할 때)

struct CalibHistoryEntry {
  std::string time, run, action;   // action: "applied" | "skipped" | "reverted"
  double speed{0}, rms_before{0}, rms_after{0}, accuracy_after{0};
  int laps{0};
};

class CalibStore {
public:
  explicit CalibStore(std::string data_dir);
  OdomCalib load() const;                         // 없으면 기본값
  void save(const OdomCalib & c) const;           // calibration.json + odometry_calibration.yaml
  void record(const CalibResult & r, const std::string & run_dir, const OdomCalib & previous, double speed, int laps, bool applied) const;
  bool revert(OdomCalib * out) const;             // 마지막 적용 이전 값으로 (성공 시 저장까지)
  // 보정 초기화: 보정 값·이력·yaml을 backup_<시각>/ 으로 옮기고 기본값(모두 1, 0)으로 시작. 주행 기록(runs)은 그대로. 반환: 백업 폴더
  std::string reset() const;
  std::vector<CalibHistoryEntry> history(std::size_t last_n = 20) const;
  const std::string & dir() const { return dir_; }

private:
  std::string dir_;
};

}  // namespace semi_slam
