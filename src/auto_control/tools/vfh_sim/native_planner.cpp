#include "roi_vfh_test/hazard_filter.hpp"
#include "roi_vfh_test/policy_state.hpp"
#include "frenet_planner.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>

using namespace roi_vfh_test;
using Clock = std::chrono::steady_clock;

int main(int argc,char** argv) {
  const bool use_vfh=argc<=1 || std::string(argv[1])!="frenet";
  struct Session {
    Planner planner;
    HazardFilter filter;
    LaneMemory lane_memory;
    ObstacleMemory obstacle_memory;
    std::optional<MotionPose> previous_pose;
    double last_avoidance{-1e9};
    double last_target{0};
    std::size_t last_use{0};
  };
  std::unordered_map<std::string,Session> sessions;
  std::size_t request_count=0;
  std::cout << std::setprecision(12);
  for (std::string line; std::getline(std::cin, line);) {
    try {
      std::istringstream input(line);
      std::string session;
      Config c;
      MotionPose pose;
      double plan_hz,trust_seconds,trust_length,trust_offset,trust_angle;
      Grid grid;
      int lane_count, hazard_count;
      input >> session >> pose.t >> pose.x >> pose.y >> pose.yaw >> c.current_speed >> c.current_steer
        >> plan_hz >> c.speed >> c.horizon >> c.step >> c.margin
        >> c.lane_width >> c.width >> c.rear >> c.front >> c.wheelbase
        >> c.max_steer >> c.steer_rate >> c.deceleration >> c.delay
        >> c.lane_lookahead >> c.reference_distance >> c.hist_low >> c.hist_high
        >> c.hazard_filter_enabled >> c.hazard_confirm_frames >> c.hazard_small_max_cells
        >> c.hazard_match_radius_cells >> c.hazard_max_gap_sec >> c.avoidance_curve_enabled
        >> c.avoidance_curve_samples >> c.avoidance_length_slack_m >> c.avoidance_lane_weight
        >> c.avoidance_steering_weight >> grid.width >> grid.height >> grid.resolution
        >> grid.y0 >> trust_seconds >> trust_length >> trust_offset >> trust_angle
        >> c.clearance_weight >> lane_count >> hazard_count;
      if (!input || plan_hz<5 || plan_hz>40 || grid.width<1 || grid.width>2000 ||
          grid.height<1 || grid.height>2000 || grid.width*grid.height>250000 ||
          lane_count<0 || lane_count>2000 || hazard_count<0 || hazard_count>grid.width*grid.height) {
        throw std::invalid_argument("invalid simulation input");
      }
      grid.cells.assign(grid.width*grid.height,-1);
      std::vector<auto_control::Point> lane_samples(lane_count);
      for (auto& point:lane_samples) input >> point.x >> point.y;
      for (int i=0; i<hazard_count; ++i) {
        int index=-1;
        if (!(input >> index) || index<0 || index>=grid.width*grid.height)
          throw std::invalid_argument("invalid terrain cell index");
        grid.cells[index]=100;
      }
      if (!input) throw std::invalid_argument("incomplete simulation input");
      input >> std::ws;
      if(!input.eof())throw std::invalid_argument("unexpected data: evaluation marks are not planner inputs");
      if (sessions.size()>=8 && !sessions.count(session)) {
        auto oldest=std::min_element(sessions.begin(),sessions.end(),[](const auto& a,const auto& b) {
          return a.second.last_use<b.second.last_use;
        });
        sessions.erase(oldest);
      }
      auto& state=sessions[session];
      state.last_use=++request_count;
      c.delay=std::max(c.delay,1/plan_hz);
      c.lane_trust_duration=trust_seconds;c.lane_trust_length=trust_length;
      c.lane_trust_offset=trust_offset;c.lane_trust_angle_deg=trust_angle;
      c.validate();
      std::vector<Point> observed;observed.reserve(lane_samples.size());
      for(auto p:lane_samples)observed.push_back({p.x,p.y});
      if (state.previous_pose && pose.t<state.previous_pose->t) {
        state.planner.reset();
        state.filter.reset();
        state.obstacle_memory.reset();state.lane_memory.reset();
        state.previous_pose.reset();
        state.last_target=0;
        state.last_avoidance=-1e9;
      }
      if (use_vfh && state.previous_pose) state.planner.rotateReference(wrap(pose.yaw-state.previous_pose->yaw));
      const auto reference=state.lane_memory.resolve(observed,pose,c);
      const bool lane_missing=!reference.trusted;
      const bool has_memory=reference.has_memory;
      const double memory_age=reference.memory_age;
      const auto& trust_state=reference.state;
      const auto& lane=reference.points;
      const auto remembered=state.lane_memory.remembered(pose);
      if(lane_missing && !has_memory)c.avoidance_lane_weight=0;
      const auto start=Clock::now();
      auto filtered=state.filter.apply(grid,pose.t,pose,c);
      auto& planning_grid=filtered.grid;
      state.obstacle_memory.augment(planning_grid,pose,c);
      const auto after_filter=Clock::now();
      auto result=use_vfh?state.planner.plan(planning_grid,lane,c,lane_missing):
        planFrenet(planning_grid,lane,c,lane_missing);
      result.status="LANE "+trust_state+" | "+result.status;
      const auto finish=Clock::now();
      // Match the default ROI launch's target-speed policy around avoidance.
      if (result.reference_blocked || (result.used_vfh && !lane_missing)) state.last_avoidance=pose.t;
      if (pose.t-state.last_avoidance<=1.0) result.speed=std::min(result.speed,1.0);
      if (state.previous_pose && state.last_target>0 && std::abs(c.current_speed)>=.05 &&
          pose.t>state.previous_pose->t) {
        result.speed=std::min(result.speed,state.last_target+.8*(pose.t-state.previous_pose->t));
      }
      state.previous_pose=pose;
      state.last_target=result.speed;
      const auto ms=[](auto duration){return std::chrono::duration<double,std::milli>(duration).count();};
      std::cout << "{\"status\":" << std::quoted(result.status)
        << ",\"speed\":" << result.speed << ",\"steer\":" << result.steer
        << ",\"laneMissing\":" << lane_missing
        << ",\"laneTrust\":" << std::quoted(trust_state)
        << ",\"hasLaneMemory\":" << has_memory << ",\"memoryAge\":" << memory_age
        << ",\"memoryPath\":[";
      for(std::size_t i=0;i<remembered.size();++i)
        std::cout<<(i?",":"")<<'['<<remembered[i].x<<','<<remembered[i].y<<']';
      std::cout << "]"
        << ",\"usedVfh\":" << result.used_vfh << ",\"fallback\":" << result.used_fallback
        << ",\"length\":" << result.length << ",\"longestCandidate\":" << result.longest_candidate
        << ",\"hazardEnds\":" << result.hazard_ends << ",\"shortCandidates\":" << result.short_candidates
        << ",\"checkedCandidates\":" << result.candidates.size() << ",\"referenceBlocked\":" << result.reference_blocked
        << ",\"filterMs\":" << ms(after_filter-start) << ",\"planMs\":" << ms(finish-after_filter)
        << ",\"raw\":" << filtered.stats.raw << ",\"confirmed\":" << filtered.stats.confirmed
        << ",\"pending\":" << filtered.stats.pending
        << ",\"rememberedHazards\":" << planning_grid.remembered_hazards.size() << ",\"path\":[";
      for (std::size_t i=0; i<result.path.size(); ++i) {
        const auto& p=result.path[i];
        std::cout << (i?",":"") << '[' << p.x << ',' << p.y << ']';
      }
      std::cout << "],\"histogram\":[";
      for (std::size_t i=0; i<result.histogram.size(); ++i)
        std::cout << (i?",":"") << '[' << result.histogram[i] << ',' << result.masked[i] << ']';
      std::cout << "],\"hazards\":[";
      bool comma=false;
      for (std::size_t i=0; i<filtered.grid.cells.size(); ++i) {
        if (!blockedTerrain(filtered.grid.cells[i])) continue;
        if (comma) std::cout << ',';
        std::cout << i;
        comma=true;
      }
      std::cout << "]}\n" << std::flush;
    } catch (const std::exception& error) {
      std::cout << "{\"error\":" << std::quoted(error.what()) << "}\n" << std::flush;
    }
  }
}
