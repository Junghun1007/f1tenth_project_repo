#include "roi_vfh_test/memory.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){
  const auto aligned=reproject({1,0},{1,1,2,pi/2,0},{1,0,0,0,0});
  check(std::abs(aligned.x-1)<1e-6 && std::abs(aligned.y-3)<1e-6,"capture-frame reprojection failed");
  MotionHistory motion;motion.add(0,.7,0);motion.add(.05,.7,0);MotionPose pose;
  check(motion.at(.025,pose)&&std::abs(pose.x-.0175)<1e-6,"speed integration/interpolation failed");
  auto generation=motion.generation;motion.add(1,.7,0);check(motion.generation>generation,"motion gap not reset");
  check(!motion.at(.5,pose),"motion extrapolated across outage");
  Grid g;g.width=10;g.height=10;g.resolution=.05;g.y0=-.25;g.cells.assign(100,-1);g.cells[55]=0;
  TerrainMemory memory;MotionPose a{1,0,0,0,0};memory.update(g,a,.5,.35);
  auto blank=g;blank.cells.assign(100,-1);
  auto fused=memory.update(blank,{1.1,.05,0,0,.05},.5,.35);
  check(fused.cells[54]==0,"memory did not compensate forward motion");
  auto hazard=blank;hazard.cells[54]=100;
  check(memory.update(hazard,{1.1,.05,0,0,.05},.5,.35).cells[54]==100,"current hazard overwritten");
  auto expired=memory.update(blank,{2,0,0,0,0},.5,.35);
  check(std::all_of(expired.cells.begin(),expired.cells.end(),[](int v){return v==-1;}),"memory time expiry failed");
  memory.clear();memory.update(g,a,.5,.35);
  auto far=memory.update(blank,{1.1,0,0,0,.4},.5,.35);
  check(std::all_of(far.cells.begin(),far.cells.end(),[](int v){return v==-1;}),"memory distance expiry failed");
  memory.clear();memory.update(g,a,.5,.35);
  auto rotated=memory.update(blank,{1.1,0,0,pi/2,0},.5,.35);
  check(rotated.at(.025,-.275)==-1,"memory extended outside map");
  memory.clear();auto turn=g;turn.cells.assign(100,-1);turn.cells[53]=0;memory.update(turn,a,.5,.35);
  const auto turned=memory.update(blank,{1.1,0,0,pi/2,0},.5,.35);
  check(turned.at(.025,-.175)==0,"memory yaw compensation failed");
  MotionHistory turning;turning.add(0,0,1);turning.add(.05,0,1);
  check(turning.at(.05,pose) && pose.y>.016,"front axle lateral motion omitted");
  std::cout<<"Memory tests passed: motion, current hazard precedence, bounded age/distance, map bounds\n";
}
