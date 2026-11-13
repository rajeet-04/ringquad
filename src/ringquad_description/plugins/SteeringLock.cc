// Ideal positive knuckle lock coupled to the arm fold mechanism.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <array>
#include <cmath>
namespace ringquad {
class SteeringLock : public gz::sim::System, public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  std::array<gz::sim::Entity,2> steering{gz::sim::kNullEntity,gz::sim::kNullEntity};
  std::array<gz::sim::Entity,2> hinges{gz::sim::kNullEntity,gz::sim::kNullEntity};
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    for(int i=0;i<2;++i) {
      const std::string corner=i==0?"fl":"fr";
      if(steering[i]==gz::sim::kNullEntity) {
        steering[i]=model.JointByName(ecm,"steer_"+corner);
        hinges[i]=model.JointByName(ecm,"hinge_"+corner);
        if(steering[i]==gz::sim::kNullEntity || hinges[i]==gz::sim::kNullEntity) continue;
        if(!ecm.Component<gz::sim::components::JointPosition>(hinges[i]))
          ecm.CreateComponent(hinges[i],gz::sim::components::JointPosition());
      }
      auto position=ecm.Component<gz::sim::components::JointPosition>(hinges[i]);
      const bool ground=position && !position->Data().empty() && position->Data()[0]>75.*M_PI/180.;
      const std::vector<gz::math::Vector2d> limits{{ground?-.63:0.,ground?.63:0.}};
      auto command=ecm.Component<gz::sim::components::JointPositionLimitsCmd>(steering[i]);
      if(command) command->Data()=limits;
      else ecm.CreateComponent(steering[i],gz::sim::components::JointPositionLimitsCmd(limits));
    }
  }
};
}
GZ_ADD_PLUGIN(ringquad::SteeringLock,gz::sim::System,
 ringquad::SteeringLock::ISystemConfigure,ringquad::SteeringLock::ISystemPreUpdate)
