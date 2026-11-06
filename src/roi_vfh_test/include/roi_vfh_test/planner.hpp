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
  double speed{.7},horizon{1.4},step{.02},wheelbase{.33},width{.30},rear{.25},front{.05};
  double margin{.04},max_steer{30*pi/180},steer_rate{240*pi/180};
  double lane_width{.65},departure{.40},deceleration{.8},delay{.20};
  double current_steer{0},current_speed{.7};
  double minimum_path_length{.12},uncertainty{.01},max_curvature{2};
  double lane_lateral_acceleration{1.6},avoidance_lateral_acceleration{.4};
  double stanley_gain{1.3},stanley_softening{.3},stanley_corner_heading{60*pi/180},stanley_opposing{.8};
  double geometry_window{.16},steering_weight{.4},control_rate{100};
  bool curvature_speed_control{true};
  double reference_distance{1.0},lane_lookahead{.05},hist_low{1.0},hist_high{2.0};
  void validate() const {
    for (double a:{speed,horizon,step,wheelbase,width,rear,front,margin,max_steer,steer_rate,
      lane_width,departure,deceleration,delay,reference_distance,lane_lookahead,hist_low,hist_high,
      minimum_path_length,uncertainty,max_curvature,lane_lateral_acceleration,avoidance_lateral_acceleration,
      stanley_gain,stanley_softening,stanley_corner_heading,stanley_opposing,geometry_window,steering_weight,control_rate})
      if (!std::isfinite(a) || a<0) throw std::invalid_argument("nonfinite/negative VFH parameter");
    if(!std::isfinite(current_speed) || std::abs(current_speed)>6 || minimum_path_length<.02 || minimum_path_length>1 ||
      uncertainty>.5 || max_curvature<.1 || max_curvature>10 || lane_lateral_acceleration<=0 ||
      avoidance_lateral_acceleration<=0 || geometry_window<=0 || steering_weight<=0 || steering_weight>1 ||
      control_rate<10 || control_rate>200 || stanley_opposing>=1 || stanley_corner_heading>pi)
      throw std::invalid_argument("Invalid source controller planning settings");
    if (!std::isfinite(current_steer) || std::abs(current_steer)>max_steer || speed<.05 || speed>3 || horizon<.3 || horizon>2.5 || step<.02 || step>.08 ||
      wheelbase<.1 || wheelbase>1 || width<.1 || width>1 || rear>1 || front>.5 || margin>.2 ||
      max_steer<.05 || max_steer>pi/3 || steer_rate<.1 || steer_rate>20 ||
      lane_width<width || lane_width>2 || departure>.4 || deceleration<.1 || deceleration>3 || delay>1 || reference_distance<.3 || lane_lookahead<.01 || lane_lookahead>.8 || hist_low>=hist_high || hist_high>1000)
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
// Follow the complete ordered lane/corridor polyline, not a single far target.
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
inline Block footprintBlock(const Grid& g,const Config& c,const Pose& p,const std::vector<std::uint8_t>* allowed=nullptr) {
  const double half=c.width/2+c.margin+c.uncertainty,cs=std::cos(p.yaw),sn=std::sin(p.yaw);
  const double back=c.rear+c.uncertainty,front=c.front+c.margin+c.uncertainty;
  const double longitudinal_half=(front+back)/2,offset=(front-back)/2;
  const double center_x=p.x+offset*cs,center_y=p.y+offset*sn,cell_half=g.resolution/2;
  const double x_radius=longitudinal_half*std::abs(cs)+half*std::abs(sn);
  const double y_radius=longitudinal_half*std::abs(sn)+half*std::abs(cs);
  const int u0=std::max(0,int(std::floor((center_x-x_radius-cell_half-g.x0)/g.resolution)));
  const int u1=std::min(g.width-1,int(std::ceil((center_x+x_radius+cell_half-g.x0)/g.resolution)));
  const int v0=int(std::floor((center_y-y_radius-cell_half-g.y0)/g.resolution));
  const int v1=int(std::ceil((center_y+y_radius+cell_half-g.y0)/g.resolution));
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
    if(v<0 || v>=g.height)return Block::Edge;
    const auto index=std::size_t(v)*g.width+u;
    if(blockedTerrain(g.cells[index]))return Block::Hazard;
    if(allowed && !(*allowed)[index])return Block::Route;
  }
  for(double a:{-back,front})for(double b:{-half,half}) {
    const double x=p.x+a*cs-b*sn,y=p.y+a*sn+b*cs;
    if(x>=0 && (x>=g.x0+g.width*g.resolution || y<g.y0 || y>=g.y0+g.height*g.resolution))return Block::Edge;
  }
  return Block::None;
}
struct Plan {
  std::vector<Candidate> candidates; std::vector<Pose> path;
  std::array<double,61> histogram{};
  std::array<bool,61> binary{},masked{};
  double speed{},steer{},length{},longest_candidate{}; bool guidance{},used_vfh{},reference_blocked{},rear_unverified{true};
  std::size_t hazard_ends{},edge_ends{},route_ends{},short_candidates{};
  std::string status{"STOP: no feasible route"};
};
// VFH+ four-stage histogram reduction, followed by Ackermann rollout verification.
// The preview explicitly reports that negative-X (rear) terrain is unobserved.
class Planner {
  std::array<bool,61> binary_{};
  bool guidance_{false}; int guide_confirm_{0},exit_confirm_{0}; double previous_angle_{0},binary_rotation_residual_{0};
public:
  void reset() {guidance_=false; guide_confirm_=exit_confirm_=0; previous_angle_=0;binary_rotation_residual_=0;binary_.fill(false);}
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
  bool guidance() const {return guidance_;}
  Plan plan(const Grid& g,const std::vector<Point>& lane,const Config& c) {
    g.validate();c.validate();Plan out;
    if(lane.size()>2000){out.status="STOP: lane sample limit";return out;}
    for(auto p:lane)if(!std::isfinite(p.x)||!std::isfinite(p.y)){out.status="STOP: invalid lane";return out;}
    const Candidate raw_lane=followReference(lane,c);
    bool lane_blocked=false;
    for(std::size_t i=1;i<raw_lane.path.size();++i) {
      const auto why=footprintBlock(g,c,raw_lane.path[i]);
      if(why!=Block::None){lane_blocked=why==Block::Hazard;break;}
    }
    const Grid& evidence=g; // Current frame only; no accumulated terrain.
    struct Band {double x,lo,hi;};std::vector<Band> bands,best_bands;
    double last_center=0;
    for(double x=.3;x<std::min(c.horizon+.5,g.x0+g.width*g.resolution);x+=g.resolution) {
      if(!bands.empty() && x-bands.back().x>.15) {
        if(best_bands.empty() || bands.back().x-bands.front().x>best_bands.back().x-best_bands.front().x)best_bands=bands;
        bands.clear();last_center=0;
      }
      double wall=0;bool have=false,found=false;Band best{};double error=1e9;
      for(int v=0;v<evidence.height;++v){
        const double y=evidence.y0+(v+.5)*evidence.resolution;
        if(evidence.at(x,y)<50)continue;
        const double gap=y-wall,center=(y+wall)/2;
        if(have && gap>=c.width+2*c.margin && gap<c.lane_width+.3 &&
          (bands.empty() || (std::abs(center-last_center)<.18 && x-bands.back().x<=.15))) {
          const double e=std::abs(center-last_center);
          bool free=true;for(double q=wall+evidence.resolution;q<y-evidence.resolution*.5;q+=evidence.resolution)if(blockedTerrain(evidence.at(x,q)))free=false;
          if(free && e<error){best={x,wall,y};error=e;found=true;}
        }
        wall=y;have=true;
      }
      if(found){bands.push_back(best);last_center=(best.lo+best.hi)/2;}
    }
    if(!best_bands.empty() && (bands.empty() || best_bands.back().x-best_bands.front().x>bands.back().x-bands.front().x))bands=std::move(best_bands);
    const bool pair_valid=bands.size()>=5 && bands.back().x-bands.front().x>=.35;
    const bool corridor=pair_valid && lane_blocked;
    std::vector<Point> reference=lane;
    if (corridor) ++guide_confirm_; else guide_confirm_=0;
    if (guide_confirm_>=3) guidance_=true;
    if (guidance_ && !pair_valid) {if (++exit_confirm_>=8) guidance_=false;} else exit_confirm_=0;
    out.guidance=guidance_ && pair_valid;
    if(lane.size()<2 && !out.guidance){out.status="STOP: lane/corridor reference unavailable";return out;}
    if(out.guidance) {
      reference.clear();for(const auto& b:bands)reference.push_back({b.x,(b.lo+b.hi)/2});
    }
    if(reference.size()<2){out.status="STOP: lost corridor";return out;}
    Point ref=reference.back();
    for(auto p:reference)if(std::hypot(p.x,p.y)>=c.reference_distance){ref=p;break;}
    const double target_angle=std::clamp(std::atan2(ref.y,std::max(.05,ref.x)),-pi/2,pi/2);
    const double half=c.width/2+c.margin+c.uncertainty, radius=half+g.resolution*.71;
    const double turn_radius=c.wheelbase/std::tan(c.max_steer);
    const double front_turn_radius=std::hypot(turn_radius,c.wheelbase);
    // Lane departure allowance is rasterized once; path sampling uses constant-time queries.
    std::vector<std::uint8_t> allowed(g.cells.size(),1);
    if (!out.guidance) for (int v=0;v<g.height;++v) for (int u=0;u<g.width;++u)
      allowed[std::size_t(v)*g.width+u]=laneDistance({g.x0+(u+.5)*g.resolution,g.y0+(v+.5)*g.resolution},lane)
        <=c.lane_width/2+c.departure;
    if(out.guidance)for(int v=0;v<g.height;++v)for(int u=0;u<g.width;++u){
      const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
      if(x<bands.front().x)continue;
      const Band* near=&bands.front();for(const auto& band:bands)if(std::abs(band.x-x)<std::abs(near->x-x))near=&band;
      allowed[std::size_t(v)*g.width+u]=std::abs(near->x-x)<=.10 && y>near->lo && y<near->hi;
    }
    const auto footprint=[&](const Pose& p) {
      const auto why=footprintBlock(g,c,p,&allowed);
      if(why==Block::Hazard)++out.hazard_ends;
      if(why==Block::Edge)++out.edge_ends;
      if(why==Block::Route)++out.route_ends;
      return why==Block::None;
    };
    const auto setSpeed=[&]() {
      const auto last=out.path.back();previous_angle_=std::atan2(last.y,last.x);
      out.speed=std::min(c.speed,-c.deceleration*c.delay+
        std::sqrt(c.deceleration*c.deceleration*c.delay*c.delay+2*c.deceleration*out.length));
      double curvature=0;for(auto p:out.path)curvature=std::max(curvature,std::abs(std::tan(p.steer)/c.wheelbase));
      if(out.used_vfh || out.guidance || c.curvature_speed_control) {
        const double lateral=(out.used_vfh || out.guidance)?c.avoidance_lateral_acceleration:c.lane_lateral_acceleration;
        out.speed=std::min(out.speed,std::sqrt(lateral/std::max(.01,curvature)));
      }
    };
    // First try tracking every bend of the lane (or the confirmed cone corridor).
    Candidate track=out.guidance?followReference(reference,c):raw_lane;
    Block reference_end=Block::None;std::size_t retained=track.path.empty()?0:1;
    for(std::size_t i=1;i<track.path.size();++i) {
      reference_end=footprintBlock(g,c,track.path[i],&allowed);
      if(reference_end!=Block::None){footprint(track.path[i]);break;}
      ++retained;
    }
    track.path.resize(retained);track.length=retained>0?(retained-1)*c.step:0;
    const double track_usable=std::max(0.,track.length-c.step);
    track.valid=track_usable+1e-9>=c.minimum_path_length && track.path.size()>2;
    out.longest_candidate=track_usable;if(!track.valid)++out.short_candidates;
    out.candidates.push_back(track);
    out.reference_blocked=reference_end==Block::Hazard || (!out.guidance && lane_blocked);
    if(!out.reference_blocked) {
      // Off-route hazards and histogram history cannot veto a clear lane curve.
      binary_.fill(false);binary_rotation_residual_=0;
      if(track.valid) {
        out.path=track.path;out.length=track_usable;out.steer=track.command_steer;setSpeed();
        out.status=out.guidance?"GUIDANCE: follow confirmed corridor":"LANE: clear lane trajectory (VFH bypass)";
      }else out.status="STOP: reference too short or outside route/map";
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
    if(targets.empty()) {
      out.status="STOP: no free VFH+ direction (hazard/turn mask)";return out;
    }
    const Polyline reference_curve(reference);
    double best_score=1e9;
    const double dt=c.step/c.speed;
    for (double angle:targets) for (int profile=0;profile<3;++profile) {
      Candidate candidate;candidate.direction=angle; Pose p;p.steer=c.current_steer; candidate.path.push_back(p);
      const Point goal{c.reference_distance*std::cos(angle),c.reference_distance*std::sin(angle)};
      for (double s=c.step;s<=c.horizon+1e-8;s+=c.step) {
        double desired;
        if (profile==0) {
          const double dx=goal.x-p.x,dy=goal.y-p.y,d2=dx*dx+dy*dy;
          desired=std::atan2(2*c.wheelbase*(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy),std::max(.1,d2));
        } else {
          const double gain=profile==1 ? 2.0 : 3.0;
          const double initial=std::atan(2*c.wheelbase*std::sin(angle)/c.reference_distance)*gain;
          desired=s<c.horizon/3 ? initial : (s<2*c.horizon/3 ? -initial : 0);
        }
        const double limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
        desired=std::clamp(desired,-limit,limit);
        if(candidate.path.size()==1)candidate.command_steer=desired;
        p.steer=steeringStep(p.steer,desired,dt,c);
        const double turn=c.step*std::tan(p.steer)/c.wheelbase;
        const double middle=p.yaw+turn/2;
        p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
        p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);p.yaw=wrap(p.yaw+turn);
        if (std::abs(p.yaw)>=pi/2 || !footprint(p)) break;
        candidate.path.push_back(p);candidate.length=s;
      }
      const double usable=std::max(0.,candidate.length-c.step);
      out.longest_candidate=std::max(out.longest_candidate,usable);
      candidate.valid=usable+1e-9>=c.minimum_path_length && candidate.path.size()>2;
      if(!candidate.valid)++out.short_candidates;
      if (candidate.valid) {
        const auto last=candidate.path.back();
        double deviation=0;for(const auto& p:candidate.path)deviation+=laneDistance({p.x,p.y},reference);
        deviation/=candidate.path.size();
        candidate.score=4*(c.horizon-candidate.length)+5*deviation+2*laneDistance({last.x,last.y},reference)+
          1.5*std::abs(wrap(angle-target_angle))+.2*std::abs(angle)+.6*std::abs(wrap(angle-previous_angle_))+
          .5*std::abs(wrap(last.yaw-reference_curve.heading({last.x,last.y})));
        if (candidate.score<best_score) {
          best_score=candidate.score;out.path=candidate.path;out.length=usable;
          out.steer=candidate.command_steer;
        }
      }
      out.candidates.push_back(std::move(candidate));
    }
    if (out.path.empty()) {
      out.status="STOP: no path >="+std::to_string(c.minimum_path_length)+"m; hazard="+std::to_string(out.hazard_ends)+
        " edge="+std::to_string(out.edge_ends)+" route="+std::to_string(out.route_ends);
      return out;
    }
    setSpeed();
    out.status=out.guidance?"GUIDANCE: VFH+ within corridor":"AVOID: VFH+ around blocked lane, return preferred";
    return out;
  }
};
}
