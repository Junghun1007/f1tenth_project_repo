#include "roi_vfh_test/policy_state.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
int main() {
  Config c;
  std::vector<Point> lane;
  for(double x=.02;x<2.9;x+=.02)lane.push_back({x,.05*x*x});
  LaneMemory memory;LaneReference reference;
  // Repeated use of one exposure is not continuous fresh evidence.
  for(int i=0;i<25;++i)reference=memory.resolve(lane,MotionPose{i*.05,0,0,0,0},c,0.);
  check(!reference.trusted && !reference.has_memory,"one exposure became a trusted lane");
  memory.reset();
  for(int i=0;i<22;++i)reference=memory.resolve(lane,MotionPose{i*.05,0,0,0,0},c);
  check(reference.trusted && reference.has_memory,"continuous lane failed confidence gate");
  const auto original=memory.remembered(MotionPose{1.05,0,0,0,0});
  const MotionPose moved{100,1,.3,.2,0};
  reference=memory.resolve({},moved,c);
  check(!reference.trusted && reference.has_memory && reference.memory_age>90,"lane memory expired or became an observation");
  const auto transformed=memory.remembered(moved);
  const auto expected=reproject(original.front(),MotionPose{1.05,0,0,0,0},moved);
  check(std::hypot(transformed.front().x-expected.x,transformed.front().y-expected.y)<1e-9,
    "remembered lane was not motion aligned");
  memory.reset();reference=memory.resolve({},moved,c);
  check(!reference.has_memory && reference.points.size()==2,"new session did not use forward preference");

  Grid g;g.width=160;g.height=60;g.resolution=.025;g.y0=-.75;g.cells.assign(9600,-1);
  g.cells[36*g.width+8]=100;
  ObstacleMemory obstacles;obstacles.augment(g,MotionPose{},c);
  check(obstacles.size()==1 && g.remembered_hazards.empty(),"front observation became a blind hazard");
  g.cells.assign(g.cells.size(),-1);
  obstacles.augment(g,MotionPose{1,.3,0,0,0},c);
  check(g.remembered_hazards.size()==1,"previous cone disappeared on entering the blind spot");
  check(g.remembered_hazards.front().x<0,"blind cone was not reprojected behind the axle");
  obstacles.augment(g,MotionPose{2,2,0,0,0},c);
  check(obstacles.size()==0 && g.remembered_hazards.empty(),"fully passed cone was retained");
  std::cout<<"Shared policy state: confidence, exposure deduplication, geometric lane memory and blind-spot retention\n";
}
