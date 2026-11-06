#include "roi_vfh_test/planner.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
int main(){
  Grid g;g.width=60;g.height=24;g.resolution=.05;g.y0=-.6;g.cells.assign(1440,0);
  std::vector<Point> lane;for(double x=.05;x<2.9;x+=.05)lane.push_back({x,0});
  Config c;Planner planner;
  auto plan=planner.plan(g,lane,c);
  check(!plan.path.empty() && plan.speed>0 && plan.speed<=.7,"open lane failed");
  check(plan.candidates.size()<=51,"candidate cap exceeded");
  check(std::abs(plan.path.back().y)<.1,"open lane swerved");
  auto unknown=g;unknown.cells.assign(1440,-1);planner.reset();
  check(planner.plan(unknown,lane,c).speed>0,"unknown terrain was blocked");
  auto cone=g;
  for(int x=18;x<=20;++x)for(int y=11;y<=12;++y)cone.cells[y*60+x]=100;
  planner.reset();plan=planner.plan(cone,lane,c);
  std::cout<<"cone path="<<plan.path.size()<<" speed="<<plan.speed<<" candidates="<<plan.candidates.size()<<"\n";
  check(plan.length>1.2,"single cone route stops before avoidance");
  check(!plan.path.empty(),"single cone did not produce a local route");
  for(auto p:plan.path)check(std::abs(p.steer)<=c.max_steer+1e-9,"steering exceeds limit");
  auto narrow=g;narrow.cells.assign(1440,100);planner.reset();
  check(planner.plan(narrow,lane,c).speed==0,"blocked route moved");
  check(planner.plan(g,{},c).speed==0,"missing ordinary lane moved");
  // Binary hysteresis preserves its prior decision within the threshold band.
  const double density=plan.histogram[30];check(density>0,"missing primary histogram");
  auto tuning=c;tuning.hist_low=density*.4;tuning.hist_high=density*.8;
  planner.reset();auto first=planner.plan(cone,lane,tuning);check(first.binary[30],"high threshold failed");
  tuning.hist_low=density*.8;tuning.hist_high=density*1.2;
  check(planner.plan(cone,lane,tuning).binary[30],"hysteresis state was discarded");
  planner.reset();check(!planner.plan(cone,lane,tuning).binary[30],"reset retained histogram state");
  // Long paired boundaries are confirmed; lane loss then uses the observed corridor.
  auto guide=g;for(int x=16;x<38;++x){guide.cells[9*60+x]=100;guide.cells[21*60+x]=100;}
  planner.reset();check(!planner.plan(guide,lane,c).guidance,"unconfirmed guidance activated");
  planner.plan(guide,lane,c);auto entered=planner.plan(guide,lane,c);
  check(entered.guidance && entered.length>1.2,"confirmed guide entry failed");
  auto lost=planner.plan(guide,{},c);check(lost.guidance && !lost.path.empty(),"corridor lost with lane labels");
  check(planner.plan(guide,{},c,&unknown).path.empty(),"memory substituted for current corridor evidence");
  auto single_wall=g;for(int x=16;x<38;++x)single_wall.cells[9*60+x]=100;
  planner.reset();for(int i=0;i<5;++i)check(!planner.plan(single_wall,lane,c).guidance,"one wall enabled guidance");
  bool threw=false;try {auto bad=c;bad.hist_low=bad.hist_high;bad.validate();}catch(...){threw=true;}
  check(threw,"invalid hysteresis accepted");
  std::cout<<"Planner tests passed\n";
}
