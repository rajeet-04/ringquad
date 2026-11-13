// Ideal positive knuckle lock coupled to the arm fold mechanism.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/Util.hh>
#include <fstream>
#include <cstdlib>
#include <sdf/Element.hh>
#include <array>
#include <cmath>
namespace ringquad {
class SteeringLock : public gz::sim::System, public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  gz::sim::Model model{gz::sim::kNullEntity};
  std::array<gz::sim::Entity,2> steering{gz::sim::kNullEntity,gz::sim::kNullEntity};
  std::array<gz::sim::Entity,2> hinges{gz::sim::kNullEntity,gz::sim::kNullEntity};
  std::ofstream audit;
  double lastAudit=-1;
  double lastFlush=-1;
  double jointLimit=.63;
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    if(sdf->HasElement("joint_limit")) jointLimit=sdf->Get<double>("joint_limit");
    if(const char *path=std::getenv("RINGQUAD_HARDWARE_LOG")) {
      audit.open(path);
      audit << "time,corner,base_z,fold,steer,steer_rate,force_x,force_y,force_z,torque_x,torque_y,torque_z\n";
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    for(int i=0;i<2;++i) {
      const std::string corner=i==0?"fl":"fr";
      if(steering[i]==gz::sim::kNullEntity) {
        steering[i]=model.JointByName(ecm,"steer_"+corner);
        hinges[i]=model.JointByName(ecm,"hinge_"+corner);
        if(steering[i]==gz::sim::kNullEntity || hinges[i]==gz::sim::kNullEntity) continue;
        if(audit.is_open()) {
          if(!ecm.Component<gz::sim::components::JointPosition>(steering[i]))
            ecm.CreateComponent(steering[i],gz::sim::components::JointPosition());
          if(!ecm.Component<gz::sim::components::JointVelocity>(steering[i]))
            ecm.CreateComponent(steering[i],gz::sim::components::JointVelocity());
          if(!ecm.Component<gz::sim::components::JointTransmittedWrench>(steering[i]))
            ecm.CreateComponent(steering[i],gz::sim::components::JointTransmittedWrench());
        }
        if(!ecm.Component<gz::sim::components::JointPosition>(hinges[i]))
          ecm.CreateComponent(hinges[i],gz::sim::components::JointPosition());
      }
      auto position=ecm.Component<gz::sim::components::JointPosition>(hinges[i]);
      const bool ground=position && !position->Data().empty() && position->Data()[0]>75.*M_PI/180.;
      const std::vector<gz::math::Vector2d> limits{{ground?-jointLimit:0.,ground?jointLimit:0.}};
      auto command=ecm.Component<gz::sim::components::JointPositionLimitsCmd>(steering[i]);
      if(command) command->Data()=limits;
      else ecm.CreateComponent(steering[i],gz::sim::components::JointPositionLimitsCmd(limits));
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if(!audit.is_open() || info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(now-lastAudit<.004) return;
    lastAudit=now;
    for(int i=0;i<2;++i) {
      auto q=ecm.Component<gz::sim::components::JointPosition>(steering[i]);
      auto v=ecm.Component<gz::sim::components::JointVelocity>(steering[i]);
      auto f=ecm.Component<gz::sim::components::JointPosition>(hinges[i]);
      auto w=ecm.Component<gz::sim::components::JointTransmittedWrench>(steering[i]);
      if(!q || q->Data().empty() || !v || v->Data().empty() || !f || f->Data().empty() || !w) continue;
      const auto &force=w->Data().force(); const auto &torque=w->Data().torque();
      audit<<now<<','<<i<<','<<gz::sim::worldPose(model.LinkByName(ecm,"base_link"),ecm).Pos().Z()<<','<<f->Data()[0]<<','<<q->Data()[0]<<','<<v->Data()[0]<<','
           <<force.x()<<','<<force.y()<<','<<force.z()<<','<<torque.x()<<','<<torque.y()<<','<<torque.z()<<'\n';
    }
    if(now-lastFlush>=1.) {audit.flush();lastFlush=now;}
  }
};
}
GZ_ADD_PLUGIN(ringquad::SteeringLock,gz::sim::System,
 ringquad::SteeringLock::ISystemConfigure,ringquad::SteeringLock::ISystemPreUpdate,ringquad::SteeringLock::ISystemPostUpdate)
