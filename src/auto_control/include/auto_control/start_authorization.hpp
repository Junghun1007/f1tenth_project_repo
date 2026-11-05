#pragma once
namespace auto_control
{
// One explicit operator decision per controller/critical-readiness session.
// A false critical input revokes approval; restoring it cannot start the car.
class StartAuthorization
{
public:
  void observe(bool critical_ready, bool manual)
  {
    if (!critical_ready || manual) {approved_=false;}
    critical_ready_=critical_ready; manual_=manual;
  }
  bool request(bool all_checks_ready)
  {
    if (!critical_ready_ || manual_ || !all_checks_ready) {return false;}
    approved_=true; return true;
  }
  void revoke() {approved_=false;}
  bool approved() const {return approved_;}
private:
  bool approved_{false}, critical_ready_{false}, manual_{false};
};
}
