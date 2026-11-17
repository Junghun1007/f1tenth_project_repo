// 브라우저 텔레메트리 JSON (web/app.js와 같은 필드). full=false면 상태·점검만 (프리뷰 꺼짐)
#pragma once
#include "cone_drive_test/core/lead_follower.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include "cone_drive_test/core/drive_state.hpp"
#include "cone_drive_test/core/stage_stats.hpp"
#include "cone_drive_test/core/system_monitor.hpp"
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct InspectionModel { std::string name, status, stage, message; double elapsed_s{0}; };
struct InspectionInfo {
  std::string attitude_status{"WAITING"}, attitude_message;
  double height_m{0}, roll_deg{0}, pitch_down_deg{0};
  std::vector<InspectionModel> models;
};
struct VehicleInfo { double speed{0}, target{0}, steer_deg{0}, cte{0}, duty{0}, brake{0}; };
struct DetectionInfo { std::string cls; float score{0}; double x{0}, y{0}, length{0}, width{0}, yaw{0}; };
struct MemoryInfo { double x{0}, y{0}; bool confirmed{false}; std::uint32_t id{0}; };

struct TelemetryFrame {
  bool full{true};
  double t{0};
  std::string state, mode, reason;
  Readiness readiness;
  InspectionInfo inspection;
  std::string stabilization{"측정 전"};   // IMU 영상 안정화 적용률 (프리뷰 꺼짐에서도 보냄)
  VehicleInfo vehicle;
  std::vector<Vec2> lane_left, lane_right, lane_center, lane_path, plan_path;
  std::string plan_mode{"center"};
  bool rollout_ok{true};
  std::vector<DetectionInfo> detections;
  std::vector<MemoryInfo> memory;
  int memory_mode{0};
  double lock_m{0};
  LeadState lead;   // 차량 추종 상태   // 모드 2: 남은 경로 고정 거리 (m)
  std::vector<StageStats::Row> stages;
  SystemUsage usage;
  std::uint64_t params_version{0};
};

std::string telemetry_json(const TelemetryFrame & f, std::size_t max_points = 40);

}  // namespace cone_drive_test
