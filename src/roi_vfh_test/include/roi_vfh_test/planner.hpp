#pragma once
#include <auto_control/control_core.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
namespace roi_vfh_test {
constexpr double pi=3.14159265358979323846;
struct Point {double x{},y{};};
struct Pose {double x{},y{},yaw{},steer{};};
struct Grid {
  int width{},height{}; double resolution{},x0{},y0{};
  std::vector<std::int8_t> cells;
  int at(double x,double y) const {
    const int u=int(std::floor((x-x0)/resolution)),v=int(std::floor((y-y0)/resolution));
    return u<0 || v<0 || u>=width || v>=height ? -1 : cells[std::size_t(v)*width+u];
  }
  void validate() const {
    if (width<1 || height<1 || width>2000 || height>2000 ||
      std::size_t(width)*height>250000 || cells.size()!=std::size_t(width)*height ||
      !std::isfinite(resolution) || resolution<.01 || resolution>.25 || !std::isfinite(x0) || !std::isfinite(y0))
      throw std::invalid_argument("invalid metric terrain grid");
  }
};
// Match the preview: mint/gray are traversable; only red occupancy (>=50) blocks.
inline bool blockedTerrain(int value) {return value>=50;}
struct Config {
  double speed{.8},horizon{1.4},step{.03},wheelbase{.33},width{.30},rear{.25},front{.05};
  double margin{.03},max_steer{30*pi/180},steer_rate{240*pi/180};
  double lane_width{.65},departure{.40},deceleration{.4},delay{.10};
  double current_steer{0},current_speed{.7};
  double minimum_path_length{.12},uncertainty{.01},max_curvature{2};
  double lane_lateral_acceleration{1.6},avoidance_lateral_acceleration{.4};
  double stanley_gain{1.3},stanley_softening{.3},stanley_corner_heading{60*pi/180},stanley_opposing{.8};
  double geometry_window{.16},steering_weight{.4},control_rate{100};
  bool curvature_speed_control{true};
  // Numeric 0/1 and integer-valued doubles share the pre-Start GUI/ROS edit path.
  double hazard_filter_enabled{1},hazard_confirm_frames{4},hazard_small_max_cells{8};
  double hazard_match_radius_cells{1},hazard_max_gap_sec{.15};
  double avoidance_curve_enabled{1},avoidance_curve_samples{9},avoidance_length_slack_m{.10};
  double avoidance_lane_weight{1},avoidance_steering_weight{1};
  double reference_distance{1.0},lane_lookahead{.05},hist_low{1.0},hist_high{2.0};
  void validate() const {
    for (double a:{speed,horizon,step,wheelbase,width,rear,front,margin,max_steer,steer_rate,
      lane_width,departure,deceleration,delay,reference_distance,lane_lookahead,hist_low,hist_high,
      minimum_path_length,uncertainty,max_curvature,lane_lateral_acceleration,avoidance_lateral_acceleration,
      stanley_gain,stanley_softening,stanley_corner_heading,stanley_opposing,geometry_window,steering_weight,control_rate})
      if (!std::isfinite(a) || a<0) throw std::invalid_argument("nonfinite/negative VFH parameter");
    for(double a:{hazard_filter_enabled,hazard_confirm_frames,hazard_small_max_cells,hazard_match_radius_cells,hazard_max_gap_sec})
      if(!std::isfinite(a))throw std::invalid_argument("nonfinite hazard filter parameter");
    if((hazard_filter_enabled!=0 && hazard_filter_enabled!=1) ||
      hazard_confirm_frames<1 || hazard_confirm_frames>5 || std::floor(hazard_confirm_frames)!=hazard_confirm_frames ||
      hazard_small_max_cells<1 || hazard_small_max_cells>50 || std::floor(hazard_small_max_cells)!=hazard_small_max_cells ||
      hazard_match_radius_cells<0 || hazard_match_radius_cells>2 || std::floor(hazard_match_radius_cells)!=hazard_match_radius_cells ||
      hazard_max_gap_sec<.02 || hazard_max_gap_sec>.5)
      throw std::invalid_argument("invalid hazard confirmation limits/integer values");
    for(double a:{avoidance_curve_enabled,avoidance_curve_samples,avoidance_length_slack_m,avoidance_lane_weight,avoidance_steering_weight})
      if(!std::isfinite(a))throw std::invalid_argument("nonfinite avoidance tuning");
    if((avoidance_curve_enabled!=0 && avoidance_curve_enabled!=1) ||
      avoidance_curve_samples<3 || avoidance_curve_samples>17 || std::floor(avoidance_curve_samples)!=avoidance_curve_samples ||
      static_cast<int>(avoidance_curve_samples)%2!=1 || avoidance_length_slack_m<0 || avoidance_length_slack_m>.5 ||
      avoidance_lane_weight<0 || avoidance_lane_weight>5 || avoidance_steering_weight<0 || avoidance_steering_weight>5)
      throw std::invalid_argument("avoidance: curves 0/1, samples odd 3..17, slack 0..0.5m, weights 0..5");
    if(!std::isfinite(current_speed) || std::abs(current_speed)>6 || minimum_path_length<.02 || minimum_path_length>1 ||
      uncertainty>.5 || max_curvature<.1 || max_curvature>10 || lane_lateral_acceleration<=0 ||
      avoidance_lateral_acceleration<=0 || geometry_window<=0 || steering_weight<=0 || steering_weight>1 ||
      control_rate<10 || control_rate>200 || stanley_opposing>=1 || stanley_corner_heading>pi)
      throw std::invalid_argument("Invalid source controller planning settings");
    if (!std::isfinite(current_steer) || std::abs(current_steer)>max_steer || speed<.05 || speed>3 || horizon<.3 || horizon>2.5 || step<.02 || step>.08 ||
      wheelbase<.1 || wheelbase>1 || width<.1 || width>1 || rear>1 || front>.5 || margin>.2 ||
      max_steer<.05 || max_steer>pi/3 || steer_rate<.1 || steer_rate>20 ||
      lane_width<width || lane_width>2 || departure>1.0 || deceleration<.1 || deceleration>3 || delay>1 || reference_distance<.3 || lane_lookahead<.01 || lane_lookahead>.8 || hist_low>=hist_high || hist_high>1000)
      throw std::invalid_argument("VFH parameter outside test limits");
  }
};
inline double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
inline double laneDistance(Point p,const std::vector<Point>& lane) {
  double best=1e9;
  for (std::size_t i=1;i<lane.size();++i) {
    const auto a=lane[i-1],b=lane[i]; const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
    if (d2<1e-9) continue;
    const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/d2,0.,1.);
    best=std::min(best,std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy));
  }
  return best;
}
struct Candidate {std::vector<Pose> path; double score{},length{},direction{},command_steer{}; bool valid{};};
// Follow the complete ordered lane polyline, not a single far target.
class Polyline {
  const std::vector<Point>& points_;
  std::vector<double> arc_;
public:
  explicit Polyline(const std::vector<Point>& points):points_(points),arc_(points.size(),0) {
    for(std::size_t i=1;i<points.size();++i)arc_[i]=arc_[i-1]+std::hypot(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
  }
  double length()const{return arc_.empty()?0:arc_.back();}
  double nearest(Point p,double minimum=0)const {
    double best=1e30,result=minimum;
    for(std::size_t i=1;i<points_.size();++i) {
      if(arc_[i]<minimum)continue;
      const auto a=points_[i-1],b=points_[i];const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
      const double span=arc_[i]-arc_[i-1];if(span<1e-9)continue;
      const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/d2,std::clamp((minimum-arc_[i-1])/span,0.,1.),1.);
      const double ex=p.x-a.x-t*dx,ey=p.y-a.y-t*dy,d=ex*ex+ey*ey;
      if(d<best){best=d;result=arc_[i-1]+t*span;}
    }
    return result;
  }
  Point at(double s)const {
    if(s<=0)return points_.front();if(s>=length())return points_.back();
    const auto upper=std::upper_bound(arc_.begin(),arc_.end(),s);
    const std::size_t i=static_cast<std::size_t>(upper-arc_.begin());
    const double t=(s-arc_[i-1])/(arc_[i]-arc_[i-1]);
    return {points_[i-1].x+t*(points_[i].x-points_[i-1].x),points_[i-1].y+t*(points_[i].y-points_[i-1].y)};
  }
  double heading(Point p)const {
    const double s=nearest(p);const auto a=at(std::max(0.,s-.025)),b=at(std::min(length(),s+.025));
    return std::atan2(b.y-a.y,b.x-a.x);
  }
};
inline double steeringStep(double current,double desired,double dt,const Config& c) {
  const double weight=1-std::pow(1-c.steering_weight,dt*c.control_rate);
  const double filtered=weight*desired+(1-weight)*current;
  return current+std::clamp(filtered-current,-c.steer_rate*dt,c.steer_rate*dt);
}
inline Candidate followReference(const std::vector<Point>& reference,const Config& c) {
  Candidate out;if(reference.size()<2)return out;
  const Polyline curve(reference);if(curve.length()<1e-6)return out;
  auto_control::OrderedPath local;local.geometry_window_m=c.geometry_window;
  local.points.resize(reference.size());local.arc_m.assign(reference.size(),0);
  for(std::size_t i=1;i<reference.size();++i)local.arc_m[i]=local.arc_m[i-1]+std::hypot(reference[i].x-reference[i-1].x,reference[i].y-reference[i-1].y);
  Pose p;p.steer=c.current_steer;out.path.push_back(p);double progress=0;
  const double dt=c.step/c.speed,limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
  for(double distance=c.step;distance<=c.horizon+1e-8;distance+=c.step) {
    progress=curve.nearest({p.x,p.y},progress);
    if(curve.length()-progress<c.step*.5)break;
    const double cs=std::cos(p.yaw),sn=std::sin(p.yaw);
    for(std::size_t i=0;i<reference.size();++i) {
      const double x=reference[i].x-p.x,y=reference[i].y-p.y;
      local.points[i]={cs*x+sn*y,-sn*x+cs*y};
    }
    const auto steering=auto_control::stanley_control(local,out.path.size()==1?c.current_speed:c.speed,
      c.stanley_gain,c.stanley_softening,c.lane_lookahead,limit,c.stanley_corner_heading,c.stanley_opposing);
    const double desired=steering.steering_angle_rad;
    if(out.path.size()==1)out.command_steer=desired;
    p.steer=steeringStep(p.steer,desired,dt,c);
    const double turn=c.step*std::tan(p.steer)/c.wheelbase;
    const double middle=p.yaw+turn/2;
    p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
    p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);p.yaw=wrap(p.yaw+turn);
    if(std::abs(p.yaw)>=pi/2)break;
    out.path.push_back(p);out.length=distance;
  }
  return out;
}
enum class Block {None,Hazard,Edge,Route};
inline Block footprintBlock(const Grid& g,const Config& c,const Pose& p,const std::vector<std::uint8_t>* allowed=nullptr,const std::vector<Point>* lane=nullptr) {
  const double half=c.width/2+c.margin+c.uncertainty,cs=std::cos(p.yaw),sn=std::sin(p.yaw);
  const double back=c.rear+c.uncertainty,front=c.front+c.margin+c.uncertainty;
  const double longitudinal_half=(front+back)/2,offset=(front-back)/2;
  const double center_x=p.x+offset*cs,center_y=p.y+offset*sn,cell_half=g.resolution/2;
  const double x_radius=longitudinal_half*std::abs(cs)+half*std::abs(sn);
  const double y_radius=longitudinal_half*std::abs(sn)+half*std::abs(cs);
  const int u0=std::max(0,int(std::floor((center_x-x_radius-cell_half-g.x0)/g.resolution)));
  const int u1=std::min(g.width-1,int(std::ceil((center_x+x_radius+cell_half-g.x0)/g.resolution)));
  const int v0=std::max(0,int(std::floor((center_y-y_radius-cell_half-g.y0)/g.resolution)));
  const int v1=std::min(g.height-1,int(std::ceil((center_y+y_radius+cell_half-g.y0)/g.resolution)));
  const double cell_projection=cell_half*(std::abs(cs)+std::abs(sn));
  for(int v=v0;v<=v1;++v)for(int u=u0;u<=u1;++u) {
    const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
    const double dx=x-center_x,dy=y-center_y;
    // Rectangle/cell separating-axis test. Cell geometry is not a second fixed
    // safety margin: use the supplied body + clearance + uncertainty once.
    if(std::abs(dx)>x_radius+cell_half || std::abs(dy)>y_radius+cell_half ||
      std::abs(dx*cs+dy*sn)>longitudinal_half+cell_projection ||
      std::abs(-dx*sn+dy*cs)>half+cell_projection)continue;
    if(x<0 || (x<=c.front && std::abs(y)<=c.width/2))continue;
    const auto index=std::size_t(v)*g.width+u;
    if(blockedTerrain(g.cells[index]))return Block::Hazard;
    if(allowed && !(*allowed)[index])return Block::Route;
  }
  // No cell outside the sensor grid means assumed free, not a map-edge wall.
  // Continue checking the departure boundary outside the grid as well, using
  // perimeter samples instead of extending the occupied/unknown map.
  if(lane && (center_x-x_radius<g.x0 || center_x+x_radius>=g.x0+g.width*g.resolution ||
      center_y-y_radius<g.y0 || center_y+y_radius>=g.y0+g.height*g.resolution)) {
    const int longitudinal_steps=std::max(1,int(std::ceil((front+back)/c.step)));
    const int lateral_steps=std::max(1,int(std::ceil(2*half/c.step)));
    const auto outsideRoute=[&](double a,double b) {
      const Point q{p.x+a*cs-b*sn,p.y+a*sn+b*cs};
      const bool outside=q.x<g.x0 || q.x>=g.x0+g.width*g.resolution ||
        q.y<g.y0 || q.y>=g.y0+g.height*g.resolution;
      return q.x>=0 && outside && laneDistance(q,*lane)>c.lane_width/2+c.departure;
    };
    for(int i=0;i<=longitudinal_steps;++i) {
      const double a=-back+(front+back)*i/longitudinal_steps;
      if(outsideRoute(a,-half) || outsideRoute(a,half))return Block::Route;
    }
    for(int i=0;i<=lateral_steps;++i) {
      const double b=-half+2*half*i/lateral_steps;
      if(outsideRoute(-back,b) || outsideRoute(front,b))return Block::Route;
    }
  }
  return Block::None;
}
struct Plan {
  std::vector<Candidate> candidates; std::vector<Pose> path;
  std::array<double,61> histogram{};
  std::array<bool,61> binary{},masked{};
  double speed{},steer{},length{},longest_candidate{}; bool used_vfh{},used_fallback{},reference_blocked{},rear_unverified{true};
  std::size_t fallback_candidates{};
  Block reference_end{Block::None};
  Pose reference_stop_pose{};
  double avoidance_trigger_distance{},reference_hazard_distance{-1};
  std::size_t hazard_ends{},edge_ends{},route_ends{},short_candidates{};
  std::string status{"STOP: no feasible route"};
};
// VFH+ four-stage histogram reduction, followed by Ackermann rollout verification.
// The preview explicitly reports that negative-X (rear) terrain is unobserved.
class Planner {
  std::array<bool,61> binary_{};
  double previous_angle_{0},previous_command_{0},binary_rotation_residual_{0};
  bool have_previous_{false};
public:
  void reset() {previous_angle_=previous_command_=binary_rotation_residual_=0;have_previous_=false;binary_.fill(false);}
  void rotateReference(double yaw) {
    previous_angle_=wrap(previous_angle_-yaw);
    binary_rotation_residual_+=yaw;
    const int shift=int(std::lround(binary_rotation_residual_/(2.5*pi/180)));
    binary_rotation_residual_-=shift*(2.5*pi/180);
    if(shift) {
      const auto old=binary_;
      for(int b=0;b<61;++b)binary_[b]=(b+shift>=0 && b+shift<61)?old[b+shift]:true;
    }
  }
  Plan plan(const Grid& g,const std::vector<Point>& lane,const Config& c) {
    g.validate();c.validate();Plan out;
    const int curve_samples=static_cast<int>(c.avoidance_curve_samples);
    const std::size_t candidate_capacity=1+51+(c.avoidance_curve_enabled!=0?(curve_samples+1)*4:0);
    out.candidates.reserve(candidate_capacity);
    if(lane.size()>2000){out.status="STOP: lane sample limit";return out;}
    for(auto p:lane)if(!std::isfinite(p.x)||!std::isfinite(p.y)){out.status="STOP: invalid lane";return out;}
    if(lane.size()<2){out.status="STOP: lane reference unavailable";return out;}
    const auto& reference=lane;
    const Polyline reference_curve(reference);
    if(reference_curve.length()<1e-6){out.status="STOP: degenerate lane reference";return out;}
    Point ref=reference.back();
    for(auto p:reference)if(std::hypot(p.x,p.y)>=c.reference_distance){ref=p;break;}
    const double target_angle=std::clamp(std::atan2(ref.y,std::max(.05,ref.x)),-pi/2,pi/2);
    const double half=c.width/2+c.margin+c.uncertainty,radius=half+g.resolution*.71;
    const double steering_limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
    const double turn_radius=c.wheelbase/std::tan(steering_limit);
    const double front_turn_radius=std::hypot(turn_radius,c.wheelbase);
    // One metric lane-distance field supports both the HARD departure boundary
    // and inexpensive SOFT return costs for every candidate sample.
    std::vector<double> lane_distance(g.cells.size());
    std::vector<std::uint8_t> allowed(g.cells.size(),1);
    for(int v=0;v<g.height;++v)for(int u=0;u<g.width;++u) {
      const auto i=std::size_t(v)*g.width+u;
      lane_distance[i]=laneDistance({g.x0+(u+.5)*g.resolution,g.y0+(v+.5)*g.resolution},lane);
      allowed[i]=lane_distance[i]<=c.lane_width/2+c.departure;
    }
    const auto distanceToLane=[&](Point p) {
      const int u=int(std::floor((p.x-g.x0)/g.resolution)),v=int(std::floor((p.y-g.y0)/g.resolution));
      if(u>=0 && v>=0 && u<g.width && v<g.height)return lane_distance[std::size_t(v)*g.width+u];
      return laneDistance(p,lane);
    };
    const auto recordEnd=[&](Block why) {
      if(why==Block::Hazard)++out.hazard_ends;
      if(why==Block::Edge)++out.edge_ends;
      if(why==Block::Route)++out.route_ends;
    };
    const auto setSpeed=[&]() {
      const auto last=out.path.back();previous_angle_=std::atan2(last.y,last.x);
      previous_command_=out.steer;have_previous_=true;
      out.speed=std::min(c.speed,-c.deceleration*c.delay+
        std::sqrt(c.deceleration*c.deceleration*c.delay*c.delay+2*c.deceleration*out.length));
      double curvature=0;for(auto p:out.path)curvature=std::max(curvature,std::abs(std::tan(p.steer)/c.wheelbase));
      if(out.used_vfh || c.curvature_speed_control) {
        const double lateral=out.used_vfh?c.avoidance_lateral_acceleration:c.lane_lateral_acceleration;
        out.speed=std::min(out.speed,std::sqrt(lateral/std::max(.01,curvature)));
      }
    };
    // This is also the return connector from an offset vehicle pose. Rejoin
    // only when the actual Stanley rollout, not just the drawn lane, is clear.
    out.avoidance_trigger_distance=c.horizon;
    Candidate track=followReference(reference,c);
    Block reference_end=Block::None;std::size_t retained=track.path.empty()?0:1;
    for(std::size_t i=1;i<track.path.size();++i) {
      reference_end=footprintBlock(g,c,track.path[i],&allowed,&lane);
      if(reference_end!=Block::None){out.reference_stop_pose=track.path[i];recordEnd(reference_end);break;}
      ++retained;
    }
    track.path.resize(retained);track.length=retained>0?(retained-1)*c.step:0;
    const double track_usable=std::max(0.,track.length-c.step);
    track.valid=track_usable+1e-9>=c.minimum_path_length && track.path.size()>2;
    out.longest_candidate=track_usable;if(!track.valid)++out.short_candidates;
    out.candidates.push_back(track);
    out.reference_end=reference_end;
    // Only an observed hazard intersecting the lane-following body sweep
    // within the full horizon activates avoidance. Route limits never request
    // an unrelated longer curve. Outside-grid space is assumed free.
    if(reference_end==Block::Hazard)out.reference_hazard_distance=retained*c.step;
    out.reference_blocked=reference_end==Block::Hazard;
    if(!out.reference_blocked) {
      binary_.fill(false);binary_rotation_residual_=0;
      if(track.valid) {
        out.path=track.path;out.length=track_usable;out.steer=track.command_steer;setSpeed();
        out.status=reference_end==Block::Route?"LANE: departure-limited tracking (VFH bypass)":
          "LANE: clear tracking/return trajectory (VFH bypass)";
      }else out.status=reference_end==Block::Edge?"STOP: lane tracking too short at grid boundary":
        (reference_end==Block::Route?"STOP: lane tracking too short at departure limit":"STOP: lane reference too short");
      return out;
    }
    out.used_vfh=true;
    double left_limit=pi,right_limit=-pi;
    for (int v=0;v<g.height;++v) for (int u=0;u<g.width;++u) {
      if (!blockedTerrain(g.cells[std::size_t(v)*g.width+u])) continue;
      const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
      if (x<0 || (x<c.front && std::abs(y)<c.width/2)) continue;
      const double d=std::hypot(x,y); if (d<.05 || d>c.horizon+.3) continue;
      const double angle=std::atan2(y,x);
      // Minimum-turn-circle masking, VFH+ stage 3. Rectangle verification follows.
      if (angle>0 && std::hypot(x+c.wheelbase,y-turn_radius)<front_turn_radius+radius) left_limit=std::min(left_limit,angle);
      if (angle<0 && std::hypot(x+c.wheelbase,y+turn_radius)<front_turn_radius+radius) right_limit=std::max(right_limit,angle);
      const double spread=std::asin(std::min(1.,radius/d));
      for (int b=0;b<61;++b) {
        const double a=(-75+2.5*b)*pi/180;
        if (std::abs(wrap(a-angle))<=spread) out.histogram[b]+=std::max(0.,1-d*d/((c.horizon+.3)*(c.horizon+.3)))*std::pow(g.resolution/.05,2);
      }
    }
    for (int b=0;b<61;++b) {
      if (out.histogram[b]>c.hist_high) binary_[b]=true;
      else if (out.histogram[b]<c.hist_low) binary_[b]=false;
      out.binary[b]=binary_[b];
      const double angle=(-75+2.5*b)*pi/180;
      out.masked[b]=binary_[b] || angle>=left_limit || angle<=right_limit;
    }
    // Keep the preferred lane direction, prior direction, and low-density valley directions.
    std::vector<double> angles;
    for (int b=0;b<61;) {
      if (out.masked[b]) {++b;continue;}
      const int first=b;while (b+1<61 && !out.masked[b+1])++b;
      const double lo=(-75+2.5*first)*pi/180,hi=(-75+2.5*b)*pi/180;
      // Narrow valley: center. Wide valley: two inset borders and in-valley references.
      if (hi-lo<=40*pi/180) angles.push_back((lo+hi)/2);
      else {
        const double a=lo+20*pi/180,z=hi-20*pi/180;
        angles.push_back(a);angles.push_back(z);
        for (double ref:{target_angle,0.,previous_angle_}) if (ref>=a && ref<=z) angles.push_back(ref);
      }
      ++b;
    }
    // Additional in-valley directions let a long Ackermann body form a lane-change curve.
    for(int b=6;b<61;b+=4)if(!out.masked[b])angles.push_back((-75+2.5*b)*pi/180);
    std::sort(angles.begin(),angles.end(),[&](double a,double b){const double da=std::abs(a-target_angle),db=std::abs(b-target_angle);return std::abs(da-db)<1e-9 ? a<b : da<db;});
    std::vector<double> targets;
    for (double a:angles) {
      bool duplicate=false; for (double b:targets) if (std::abs(a-b)<.04) duplicate=true;
      if (!duplicate) targets.push_back(a);
      if (targets.size()>=17) break;
    }
    const bool all_masked=targets.empty();
    const double dt=c.step/c.speed;
    std::vector<bool> fallback;fallback.reserve(candidate_capacity);fallback.push_back(false); // Parallel to candidates, including reference.
    // All alternatives use the SAME rectangle/cell, grid and departure checks.
    const auto rollout=[&](auto steering,bool is_fallback,double direction) {
      Candidate candidate;candidate.direction=direction;candidate.path.reserve(int(c.horizon/c.step)+1);
      Pose p;p.steer=c.current_steer;candidate.path.push_back(p);
      for(double s=c.step;s<=c.horizon+1e-8;s+=c.step) {
        const double desired=std::clamp(steering(p,s),-steering_limit,steering_limit);
        if(candidate.path.size()==1)candidate.command_steer=desired;
        p.steer=steeringStep(p.steer,desired,dt,c);
        const double turn=c.step*std::tan(p.steer)/c.wheelbase,middle=p.yaw+turn/2;
        p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
        p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);p.yaw=wrap(p.yaw+turn);
        if(std::abs(p.yaw)>=pi/2 || p.x<0)break;
        const auto why=footprintBlock(g,c,p,&allowed,&lane);
        if(why!=Block::None){recordEnd(why);break;}
        candidate.path.push_back(p);candidate.length=s;
      }
      const double usable=std::max(0.,candidate.length-c.step);
      out.longest_candidate=std::max(out.longest_candidate,usable);
      candidate.valid=usable+1e-9>=c.minimum_path_length && candidate.path.size()>2;
      if(!candidate.valid)++out.short_candidates;
      if(is_fallback)++out.fallback_candidates;
      out.candidates.push_back(std::move(candidate));fallback.push_back(is_fallback);
    };
    // Preferred VFH+ valleys and their three original steering profiles.
    for(double angle:targets)for(int profile=0;profile<3;++profile) {
      const Point goal{c.reference_distance*std::cos(angle),c.reference_distance*std::sin(angle)};
      rollout([&](const Pose& p,double s) {
        if(profile==0) {
          const double dx=goal.x-p.x,dy=goal.y-p.y,d2=dx*dx+dy*dy;
          return std::atan2(2*c.wheelbase*(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy),std::max(.1,d2));
        }
        const double initial=std::atan(2*c.wheelbase*std::sin(angle)/c.reference_distance)*(profile==1?2.:3.);
        return s<c.horizon/3?initial:(s<2*c.horizon/3?-initial:0.);
      },false,angle);
    }
    // A blocked polar ray does not prove every curved path collides. These
    // bounded extra profiles are still subject to the complete footprint check.
    if(c.avoidance_curve_enabled!=0) {
      std::vector<double> seeds;seeds.reserve(curve_samples+1);
      const int half_samples=(curve_samples-1)/2;
      for(int i=-half_samples;i<=half_samples;++i)seeds.push_back(steering_limit*i/half_samples);
      const double current=std::clamp(c.current_steer,-steering_limit,steering_limit);
      if(std::none_of(seeds.begin(),seeds.end(),[&](double a){return std::abs(a-current)<1e-4;}))seeds.push_back(current);
      for(double initial:seeds)for(int profile=0;profile<4;++profile) {
        rollout([&](const Pose&,double s) {
          if(profile==3)return s<std::min(.20,c.horizon*.2)?0.:initial;
          if(profile==0 || s<c.horizon*.4)return initial;
          if(profile==1 || s>=c.horizon*.8)return 0.;
          return -initial;
        },true,std::atan(std::tan(initial)*c.reference_distance/c.wheelbase));
      }
    }
    double longest=0;
    for(std::size_t i=1;i<out.candidates.size();++i)if(out.candidates[i].valid)
      longest=std::max(longest,std::max(0.,out.candidates[i].length-c.step));
    // Progress first: lane proximity must not select a short dead-end prefix
    // instead of a substantially longer feasible detour. Within the configured
    // length slack, use soft lane-return and steering-continuity costs.
    const double length_slack=c.avoidance_length_slack_m;
    double best_score=std::numeric_limits<double>::infinity();std::size_t best_index=0;
    for(std::size_t i=1;i<out.candidates.size();++i) {
      auto& candidate=out.candidates[i];if(!candidate.valid)continue;
      const double usable=std::max(0.,candidate.length-c.step);
      if(usable+length_slack+1e-9<longest)continue;
      const auto last=candidate.path.back();
      double deviation=0;for(const auto& p:candidate.path)deviation+=distanceToLane({p.x,p.y});
      deviation/=candidate.path.size();
      const double lane_cost=1.2*deviation+.8*distanceToLane({last.x,last.y})+
        .35*std::abs(wrap(last.yaw-reference_curve.heading({last.x,last.y})));
      const double continuity_cost=.35*std::abs(candidate.command_steer-c.current_steer)/steering_limit+
        (have_previous_?.2*std::abs(candidate.command_steer-previous_command_)/steering_limit+
          .25*std::abs(wrap(std::atan2(last.y,last.x)-previous_angle_)):0.);
      candidate.score=2*(longest-usable)+c.avoidance_lane_weight*lane_cost+
        c.avoidance_steering_weight*continuity_cost+(fallback[i]?.03:0.)-.3*last.x;
      if(candidate.score<best_score) {
        best_score=candidate.score;best_index=i;
      }
    }
    if(best_index==0) {
      out.status="STOP: no collision-free curve >="+std::to_string(c.minimum_path_length)+"m; hazard="+
        std::to_string(out.hazard_ends)+" edge="+std::to_string(out.edge_ends)+" route="+std::to_string(out.route_ends);
      return out;
    }
    const auto& best=out.candidates[best_index];
    out.path=best.path;out.length=std::max(0.,best.length-c.step);
    out.steer=best.command_steer;out.used_fallback=fallback[best_index];
    setSpeed();
    out.status=out.used_fallback?(all_masked?"AVOID: checked curve (VFH fully masked); lane return preferred":
      "AVOID: checked curve; lane return preferred"):"AVOID: VFH+ valley; lane return preferred";
    return out;
  }
};
}
