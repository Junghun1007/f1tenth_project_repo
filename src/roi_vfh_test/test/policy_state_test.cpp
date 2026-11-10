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

  // Disabling storage must also discard an already remembered lane.
  for(int i=0;i<22;++i)reference=memory.resolve(lane,MotionPose{i*.05,0,0,0,0},c);
  check(reference.has_memory,"lane memory setup failed");
  c.lane_memory_enabled=0;
  reference=memory.resolve({},MotionPose{2,0,0,0,0},c);
  check(!reference.has_memory && memory.remembered(MotionPose{}).empty(),"disabled lane storage retained geometry");
  for(int i=0;i<22;++i)reference=memory.resolve(lane,MotionPose{3+i*.05,0,0,0,0},c);
  check(reference.trusted && !reference.has_memory,"disabling lane storage disabled current-lane validation");
  c.lane_memory_enabled=1;
  reference=memory.resolve({},MotionPose{5,0,0,0,0},c);
  check(!reference.has_memory,"reenabling lane storage resurrected old geometry");
  std::cout<<"Shared policy state: confidence, exposure deduplication and optional geometric lane memory\n";
}
