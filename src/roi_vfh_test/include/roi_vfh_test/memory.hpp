#pragma once
#include "roi_vfh_test/planner.hpp"
#include <deque>
#include <unordered_map>
namespace roi_vfh_test {
struct MotionPose {double t{},x{},y{},yaw{},distance{};};
inline Point reproject(Point p,const MotionPose& from,const MotionPose& to){
  const double wx=from.x+std::cos(from.yaw)*p.x-std::sin(from.yaw)*p.y;
  const double wy=from.y+std::sin(from.yaw)*p.x+std::cos(from.yaw)*p.y;
  const double dx=wx-to.x,dy=wy-to.y;
  return {std::cos(to.yaw)*dx+std::sin(to.yaw)*dy,-std::sin(to.yaw)*dx+std::cos(to.yaw)*dy};
}
class MotionHistory {
  std::deque<MotionPose> poses_;double last_speed_{},last_yaw_rate_{};
public:
  double front_axle_offset{.33}; // VESC longitudinal speed is modeled at the rear axle.
  std::uint64_t generation{0};
  void clear(){poses_.clear();++generation;}
  void add(double t,double speed,double yaw_rate){
    if(!std::isfinite(t)||!std::isfinite(speed)||!std::isfinite(yaw_rate)||std::abs(speed)>6||std::abs(yaw_rate)>8){clear();return;}
    if(poses_.empty()){poses_.push_back({t,0,0,0,0});last_speed_=speed;last_yaw_rate_=yaw_rate;return;}
    const auto p=poses_.back();const double dt=t-p.t;
    if(dt<=0 || dt>.1){clear();poses_.push_back({t,0,0,0,0});last_speed_=speed;last_yaw_rate_=yaw_rate;return;}
    const double v=(speed+last_speed_)/2,w=(yaw_rate+last_yaw_rate_)/2,angle=p.yaw+w*dt/2;
    const double lateral=front_axle_offset*w;
    poses_.push_back({t,p.x+dt*(v*std::cos(angle)-lateral*std::sin(angle)),p.y+dt*(v*std::sin(angle)+lateral*std::cos(angle)),wrap(p.yaw+w*dt),p.distance+std::hypot(v,lateral)*dt});
    last_speed_=speed;last_yaw_rate_=yaw_rate;
    while(poses_.size()>1500 || poses_.front().t<t-3)poses_.pop_front();
  }
  bool at(double t,MotionPose& out)const{
    if(poses_.empty() || t<poses_.front().t || t>poses_.back().t+.03)return false;
    for(std::size_t i=1;i<poses_.size();++i)if(t<=poses_[i].t){
      const auto a=poses_[i-1],b=poses_[i];const double f=(t-a.t)/(b.t-a.t);
      out={t,a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),wrap(a.yaw+f*wrap(b.yaw-a.yaw)),a.distance+f*(b.distance-a.distance)};return true;
    }
    const auto p=poses_.back();const double dt=t-p.t;
    const double angle=p.yaw+last_yaw_rate_*dt/2,lateral=front_axle_offset*last_yaw_rate_;
    out={t,p.x+dt*(last_speed_*std::cos(angle)-lateral*std::sin(angle)),p.y+dt*(last_speed_*std::sin(angle)+lateral*std::cos(angle)),wrap(p.yaw+last_yaw_rate_*dt),p.distance+std::hypot(last_speed_,lateral)*dt};return true;
  }
};
class TerrainMemory {
  struct Cell{double x,y,t,distance;std::int8_t value;};
  struct Key{int x,y;bool operator==(const Key& k)const{return x==k.x&&y==k.y;}};
  struct Hash{std::size_t operator()(const Key& k)const{return std::hash<int>{}(k.x)^(std::hash<int>{}(k.y)<<1);}};
  std::unordered_map<Key,Cell,Hash> cells_;
  double resolution_{0};
public:
  void clear(){cells_.clear();resolution_=0;}
  Grid update(const Grid& raw,const MotionPose& pose,double ttl,double max_distance){
    raw.validate();if(resolution_!=raw.resolution){clear();resolution_=raw.resolution;}
    const double cs=std::cos(pose.yaw),sn=std::sin(pose.yaw);
    for(auto it=cells_.begin();it!=cells_.end();) {
      if(pose.t<it->second.t || pose.t-it->second.t>ttl || pose.distance-it->second.distance>max_distance)it=cells_.erase(it);else ++it;
    }
    Grid out=raw;
    // Observed current cells always win. Memory never extends the current map footprint.
    for(const auto& entry:cells_){const auto& c=entry.second;
      const double dx=c.x-pose.x,dy=c.y-pose.y,x=cs*dx+sn*dy,y=-sn*dx+cs*dy;
      const int u=int(std::floor((x-raw.x0)/raw.resolution)),v=int(std::floor((y-raw.y0)/raw.resolution));
      if(u<0||v<0||u>=raw.width||v>=raw.height)continue;
      const auto index=std::size_t(v)*raw.width+u;
      if(raw.cells[index]==-1 && (out.cells[index]==-1 || c.value>=50))out.cells[index]=c.value;
    }
    std::unordered_map<Key,Cell,Hash> fresh;
    for(int v=0;v<raw.height;++v)for(int u=0;u<raw.width;++u){
      const auto value=raw.cells[std::size_t(v)*raw.width+u];if(value!=0&&value<50)continue;
      const double x=raw.x0+(u+.5)*raw.resolution,y=raw.y0+(v+.5)*raw.resolution;
      const double wx=pose.x+cs*x-sn*y,wy=pose.y+sn*x+cs*y;
      const Key key{int(std::floor(wx/raw.resolution)),int(std::floor(wy/raw.resolution))};
      const auto found=fresh.find(key);
      if(found==fresh.end() || value>=50)fresh[key]={wx,wy,pose.t,pose.distance,value};
    }
    for(const auto& c:fresh)cells_[c.first]=c.second;
    if(cells_.size()>100000)clear();
    return out;
  }
};
}
