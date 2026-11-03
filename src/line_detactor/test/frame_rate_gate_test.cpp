#include "line_detactor/frame_rate_gate.hpp"
#include <cassert>
#include <limits>

int main()
{
  line_detactor::FrameRateGate gate;
  int count=0;
  for (int i=0;i<800;++i) {count+=gate.accept(i/80.0,5.0);}
  assert(count==50);  // 80Hz source -> exactly 5Hz over ten seconds
  assert(gate.accept(10.,80.));  // transition is immediate
  assert(gate.accept(10.0125,80.));
  assert(!gate.accept(10.013,80.));
  assert(gate.accept(20.,5.));  // no backlog burst after a long gap
  assert(!gate.accept(20.001,5.));
  assert(gate.accept(20.002,0.));  // legacy unlimited mode
  bool rejected=false;
  try {gate.accept(21.,std::numeric_limits<double>::infinity());}
  catch (const std::invalid_argument &) {rejected=true;}
  assert(rejected);
}
