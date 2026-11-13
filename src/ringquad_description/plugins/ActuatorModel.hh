#pragma once
#include <algorithm>
#include <cmath>
namespace ringquad {
inline double SpeedTorque(double speed, double target, double inertia,
                          double tau, double dragCoefficient, double limit) {
  if(!std::isfinite(speed) || !std::isfinite(target)) return 0.;
  return std::clamp(inertia*(target-speed)/tau+dragCoefficient*speed*std::abs(speed),-limit,limit);
}
}
