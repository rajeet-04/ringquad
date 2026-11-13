#include "../src/ringquad_description/plugins/ActuatorModel.hh"
#include <cassert>
#include <cmath>
int main(){
  for(double target:{-1090.,-800.,0.,800.,1090.}) {
    double speed=0.;
    for(int i=0;i<10000;++i) {
      double drag=8.4e-6*.016*speed*std::abs(speed);
      double torque=ringquad::SpeedTorque(speed,target,1.1e-4,.0125,8.4e-6*.016,.33);
      assert(std::abs(torque)<=.33);
      speed+=(torque-drag)/1.1e-4*.001;
    }
    assert(std::abs(speed-target)<.1);
  }
  assert(ringquad::SpeedTorque(0.,NAN,1.,1.,0.,.33)==0.);
}
