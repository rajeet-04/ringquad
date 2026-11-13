#pragma once
#include <algorithm>
#include <cmath>
namespace ringquad {
struct FiniteLatch {
  double stiffness=300., damping=.6, backlash=.000872664626, breakTorque=8.;
  bool broken=false;
  double Update(bool engaged, double position, double velocity) {
    if(!std::isfinite(position) || !std::isfinite(velocity)) broken=true;
    if(broken || !engaged) return 0.;
    const double error=std::copysign(std::max(0.,std::abs(position)-backlash),position);
    const double torque=-stiffness*error-damping*velocity;
    if(!std::isfinite(torque) || std::abs(torque)>breakTorque) {broken=true;return 0.;}
    return torque;
  }
};
}
