#include "roi_vfh_test/hazard_filter.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace roi_vfh_test;
using Clock = std::chrono::steady_clock;

int main() {
  struct Session {
    Planner planner;
    HazardFilter filter;
    std::optional<MotionPose> previous_pose;
    std::vector<Point> trusted_world,pending_world;
    double pending_since{-1},pending_at{-1},trusted_at{-1};
    Point trusted_pose;
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
      double plan_hz,trust_seconds,trust_length,trust_offset,trust_angle,fade_seconds,fade_distance;
      Grid grid;
      int lane_count, hazard_count;
      input >> session >> pose.t >> pose.x >> pose.y >> pose.yaw >> c.current_speed >> c.current_steer
        >> plan_hz >> c.speed >> c.horizon >> c.step >> c.margin >> c.departure
        >> c.lane_width >> c.width >> c.rear >> c.front >> c.wheelbase
        >> c.max_steer >> c.steer_rate >> c.deceleration >> c.delay
        >> c.lane_lookahead >> c.reference_distance >> c.hist_low >> c.hist_high
        >> c.hazard_filter_enabled >> c.hazard_confirm_frames >> c.hazard_small_max_cells
        >> c.hazard_match_radius_cells >> c.hazard_max_gap_sec >> c.avoidance_curve_enabled
        >> c.avoidance_curve_samples >> c.avoidance_length_slack_m >> c.avoidance_lane_weight
        >> c.avoidance_steering_weight >> grid.width >> grid.height >> grid.resolution
        >> grid.y0 >> trust_seconds >> trust_length >> trust_offset >> trust_angle
        >> fade_seconds >> fade_distance >> lane_count >> hazard_count;
      for(double value:{trust_seconds,trust_length,trust_offset,trust_angle,fade_seconds,fade_distance})
        if(!std::isfinite(value) || value<=0 || value>30)throw std::invalid_argument("invalid lane trust settings");
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
      if (sessions.size()>=8 && !sessions.count(session)) {
        auto oldest=std::min_element(sessions.begin(),sessions.end(),[](const auto& a,const auto& b) {
          return a.second.last_use<b.second.last_use;
        });
        sessions.erase(oldest);
      }
      auto& state=sessions[session];
      state.last_use=++request_count;
      c.delay=std::max(c.delay,1/plan_hz);
      const auto ordered=auto_control::build_ordered_path(lane_samples,8,c.minimum_path_length,.01,2.98,.15,c.geometry_window);
      std::vector<Point> lane;
      if (ordered) for (const auto& point:ordered->points) lane.push_back({point.x,point.y});
      if (state.previous_pose && pose.t<state.previous_pose->t) {
        state.planner.reset();
        state.filter.reset();
        state.previous_pose.reset();
        state.trusted_world.clear();state.pending_world.clear();state.pending_since=-1;state.trusted_at=-1;
        state.last_target=0;
        state.last_avoidance=-1e9;
      }
      if (state.previous_pose) state.planner.rotateReference(wrap(pose.yaw-state.previous_pose->yaw));
      const auto worldPoint=[&](Point p) {
        return Point{pose.x+std::cos(pose.yaw)*p.x-std::sin(pose.yaw)*p.y,
          pose.y+std::sin(pose.yaw)*p.x+std::cos(pose.yaw)*p.y};
      };
      const auto localPoint=[&](Point p) {
        const double x=p.x-pose.x,y=p.y-pose.y;
        return Point{std::cos(pose.yaw)*x+std::sin(pose.yaw)*y,
          -std::sin(pose.yaw)*x+std::cos(pose.yaw)*y};
      };
      bool lane_trusted=false;
      std::string trust_state="missing";
      if(ordered && Polyline(lane).length()>=trust_length) {
        std::vector<Point> world;world.reserve(lane.size());
        for(auto p:lane)world.push_back(worldPoint(p));
        bool consistent=!state.pending_world.empty() && pose.t-state.pending_at<=.15;
        if(consistent) {
          const Polyline prior(state.pending_world);
          const auto probe=worldPoint(lane.front());
          consistent=laneDistance(probe,state.pending_world)<=trust_offset &&
            std::abs(wrap(prior.heading(probe)-(pose.yaw+ordered->heading(c.lane_lookahead))))<=trust_angle*pi/180;
        }
        if(!consistent)state.pending_since=pose.t;
        state.pending_world=std::move(world);state.pending_at=pose.t;
        lane_trusted=pose.t-state.pending_since+1e-9>=trust_seconds;
        trust_state=lane_trusted?"trusted":"confirming";
        if(lane_trusted) {
          state.trusted_world=state.pending_world;state.trusted_at=pose.t;
          state.trusted_pose={pose.x,pose.y};
        }
      }else {
        state.pending_world.clear();state.pending_since=-1;
        if(ordered)trust_state="too short";
      }
      const bool lane_missing=!lane_trusted;
      const double memory_age=state.trusted_at<0?0:pose.t-state.trusted_at;
      const double memory_distance=std::hypot(pose.x-state.trusted_pose.x,pose.y-state.trusted_pose.y);
      const double memory_weight=state.trusted_at<0?0:std::exp(-memory_age/fade_seconds-memory_distance/fade_distance);
      std::vector<Point> remembered;
      for(auto p:state.trusted_world)remembered.push_back(localPoint(p));
      if(!lane_trusted) {
        lane=remembered;
        if(lane.size()>=2) {
          const auto last=lane.back(),previous=lane[lane.size()-2];
          const double heading=std::atan2(last.y-previous.y,last.x-previous.x);
          if(std::cos(heading)>.15 && last.x<c.horizon+c.reference_distance) {
            const double extra=(c.horizon+c.reference_distance-last.x)/std::cos(heading);
            lane.push_back({last.x+extra*std::cos(heading),last.y+extra*std::sin(heading)});
          }
        }
        if(lane.size()<2 || memory_weight<.01)lane={{0,0},{std::max(c.horizon,c.reference_distance),0}};
      }
      // Only trusted CURRENT observations permit direct lane tracking.
      // Stored geometry is a soft avoidance preference and never a hard bound.
      c.avoidance_lane_weight*=lane_trusted?1:memory_weight;
      const auto start=Clock::now();
      const auto filtered=state.filter.apply(grid,pose.t,pose,c);
      const auto after_filter=Clock::now();
      auto result=state.planner.plan(filtered.grid,lane,c,lane_missing);
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
        << ",\"memoryWeight\":" << memory_weight << ",\"memoryAge\":" << memory_age
        << ",\"memoryPath\":[";
      for(std::size_t i=0;i<remembered.size();++i)
        std::cout<<(i?",":"")<<'['<<remembered[i].x<<','<<remembered[i].y<<']';
      std::cout << "],\"freeDetour\":" << result.free_detour
        << ",\"usedVfh\":" << result.used_vfh << ",\"fallback\":" << result.used_fallback
        << ",\"filterMs\":" << ms(after_filter-start) << ",\"planMs\":" << ms(finish-after_filter)
        << ",\"raw\":" << filtered.stats.raw << ",\"confirmed\":" << filtered.stats.confirmed
        << ",\"pending\":" << filtered.stats.pending << ",\"path\":[";
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
