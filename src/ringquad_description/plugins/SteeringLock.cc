// Finite motor, fold and steering actuators, breakable latch and native odometry.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <set>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>
#include <atomic>
#include <functional>
#include "FiniteLatch.hh"
#include "ActuatorModel.hh"
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
  std::ofstream contactAudit;
  bool contactInitialized=false;
  double lastContact=-1.;
  double lastAudit=-1;
  double lastFlush=-1;
  double jointLimit=.63;
  double servoTorque=1.;
  double servoSpeed=1.5;
  std::array<double,2> servoReference{0.,0.};
  std::atomic<double> foldTarget{0.};
  std::array<std::atomic<double>,2> target{0.,0.};
  double lastFaultTime=-1.;
  double lastOdomTime=-1.;
  gz::sim::Entity baseLink=gz::sim::kNullEntity;
  gz::transport::Node::Publisher odomPublisher;
  bool lastFault=false;
  std::array<FiniteLatch,2> latches;
  std::array<gz::sim::Entity,4> wheels{0,0,0,0};
  std::array<gz::sim::Entity,4> foldJoints{0,0,0,0};
  std::array<gz::sim::Entity,4> propJoints{0,0,0,0};
  std::array<std::atomic<double>,4> propTarget{0.,0.,0.,0.};
  std::array<std::atomic<double>,4> wheelTarget{0.,0.,0.,0.};
  std::array<double,2> latchTorque{0.,0.};
  gz::transport::Node transport;
  gz::transport::Node::Publisher faultPublisher;
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    model=gz::sim::Model(entity);
    odomPublisher=transport.Advertise<gz::msgs::Odometry>("/odom");
    std::function<void(const gz::msgs::Actuators &)> wheelCallback=[this](const gz::msgs::Actuators &msg) {
      for(int i=0;i<4;++i) wheelTarget[i].store(i<msg.velocity_size() && std::isfinite(msg.velocity(i))?std::clamp(msg.velocity(i),-5.6,5.6):0.);
    };
    transport.Subscribe("/ringquad/wheel_speed",wheelCallback);
    std::function<void(const gz::msgs::Actuators &)> propCallback=[this](const gz::msgs::Actuators &msg) {
      for(int i=0;i<4;++i) propTarget[i].store(i<msg.velocity_size() && std::isfinite(msg.velocity(i))?std::clamp(msg.velocity(i),0.,1090.):0.);
    };
    transport.Subscribe("/ringquad/motor_speed",propCallback);
    std::function<void(const gz::msgs::Double &)> foldCallback=[this](const gz::msgs::Double &msg){foldTarget.store(std::isfinite(msg.data())?std::clamp(msg.data(),0.,M_PI/2):0.);};
    transport.Subscribe("/ringquad/fold",foldCallback);
    faultPublisher=transport.Advertise<gz::msgs::Boolean>("/ringquad/lock_fault");
    if(sdf->HasElement("servo_torque")) servoTorque=sdf->Get<double>("servo_torque");
    if(sdf->HasElement("servo_speed")) servoSpeed=sdf->Get<double>("servo_speed");
    for(int i=0;i<2;++i) {
      const std::string topic=i==0?"/ringquad/steer_fl":"/ringquad/steer_fr";
      std::function<void(const gz::msgs::Double &)> callback=[this,i](const gz::msgs::Double &msg) {target[i].store(std::isfinite(msg.data())?msg.data():0.);};
      transport.Subscribe(topic,callback);
    }
    for(auto &latch:latches) {
      if(sdf->HasElement("lock_break_torque")) latch.breakTorque=sdf->Get<double>("lock_break_torque");
    }
    if(sdf->HasElement("joint_limit")) jointLimit=sdf->Get<double>("joint_limit");
    if(const char *path=std::getenv("RINGQUAD_CONTACT_LOG")) {contactAudit.open(path);contactAudit<<"time,collision1,collision2,depth_m,point_z,ring_z,axis_z,fold_command,normal_force\n";}
    if(const char *path=std::getenv("RINGQUAD_HARDWARE_LOG")) {
      audit.open(path);
      audit << "time,corner,base_z,fold,steer,steer_rate,force_x,force_y,force_z,torque_x,torque_y,torque_z,lock_torque,lock_broken,fold_velocity\n";
    }
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    if(baseLink==gz::sim::kNullEntity) {baseLink=model.LinkByName(ecm,"base_link");if(baseLink!=gz::sim::kNullEntity) gz::sim::Link(baseLink).EnableVelocityChecks(ecm,true);}
    if(contactAudit.is_open() && !contactInitialized) {
      std::vector<gz::sim::Entity> collisions;
      ecm.Each<gz::sim::components::Collision>([&](const gz::sim::Entity &entity,const gz::sim::components::Collision *) {collisions.push_back(entity);return true;});
      for(auto entity:collisions) if(!ecm.Component<gz::sim::components::ContactSensorData>(entity)) ecm.CreateComponent(entity,gz::sim::components::ContactSensorData());
      contactInitialized=true;
    }
    for(int i=0;i<2;++i) {
      const std::string corner=i==0?"fl":"fr";
      if(steering[i]==gz::sim::kNullEntity) {
        steering[i]=model.JointByName(ecm,"steer_"+corner);
        hinges[i]=model.JointByName(ecm,"hinge_"+corner);
        if(steering[i]==gz::sim::kNullEntity || hinges[i]==gz::sim::kNullEntity) continue;
        {
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
      // Mechanical travel stays finite; the latch resists with a spring and can break.
      const std::vector<gz::math::Vector2d> limits{{-jointLimit,jointLimit}};
      auto q=ecm.Component<gz::sim::components::JointPosition>(steering[i]);
      auto v=ecm.Component<gz::sim::components::JointVelocity>(steering[i]);
      if(q && v && !q->Data().empty() && !v->Data().empty()) {
        latchTorque[i]=latches[i].Update(!ground,q->Data()[0],v->Data()[0]);
        const double command=ground?std::clamp(target[i].load(),-jointLimit,jointLimit):0.;
        const double step=servoSpeed*std::chrono::duration<double>(info.dt).count();
        servoReference[i]+=std::clamp(command-servoReference[i],-step,step);
        const double servo=std::clamp(100.*(servoReference[i]-q->Data()[0])-.3*v->Data()[0],-servoTorque,servoTorque);
        const double total=servo+latchTorque[i];
        auto force=ecm.Component<gz::sim::components::JointForceCmd>(steering[i]);
        if(force && !force->Data().empty()) force->Data()[0]=total;
        else if(force) force->Data()={total};
        else ecm.CreateComponent(steering[i],gz::sim::components::JointForceCmd({total}));
      }
      auto command=ecm.Component<gz::sim::components::JointPositionLimitsCmd>(steering[i]);
      if(command) command->Data()=limits;
      else ecm.CreateComponent(steering[i],gz::sim::components::JointPositionLimitsCmd(limits));
    }
    // Wheel drives apply bounded torque, never an ideal velocity constraint.
    const std::array<std::string,4> corners{"fl","fr","rl","rr"};
    const std::array<double,4> direction{1.,-1.,-1.,1.};
    for(int i=0;i<4;++i) {
      if(foldJoints[i]==gz::sim::kNullEntity) {
        foldJoints[i]=model.JointByName(ecm,"hinge_"+corners[i]);
        if(foldJoints[i]==gz::sim::kNullEntity) continue;
        if(!ecm.Component<gz::sim::components::JointVelocity>(foldJoints[i])) ecm.CreateComponent(foldJoints[i],gz::sim::components::JointVelocity());
        if(!ecm.Component<gz::sim::components::JointPosition>(foldJoints[i])) ecm.CreateComponent(foldJoints[i],gz::sim::components::JointPosition());
      }
      auto fq=ecm.Component<gz::sim::components::JointPosition>(foldJoints[i]);
      auto fv=ecm.Component<gz::sim::components::JointVelocity>(foldJoints[i]);
      if(fq && fv && !fq->Data().empty() && !fv->Data().empty()) {
        const double effort=std::clamp(100.*(foldTarget.load()-fq->Data()[0])-.2*fv->Data()[0],-10.,10.);
        auto force=ecm.Component<gz::sim::components::JointForceCmd>(foldJoints[i]);
        if(force) force->Data()={effort};
        else ecm.CreateComponent(foldJoints[i],gz::sim::components::JointForceCmd({effort}));
      }
      if(wheels[i]==gz::sim::kNullEntity) {
        wheels[i]=model.JointByName(ecm,"wheel_"+corners[i]);
        if(wheels[i]==gz::sim::kNullEntity) continue;
        if(!ecm.Component<gz::sim::components::JointVelocity>(wheels[i])) ecm.CreateComponent(wheels[i],gz::sim::components::JointVelocity());
      }
      auto velocity=ecm.Component<gz::sim::components::JointVelocity>(wheels[i]);
      if(!velocity || velocity->Data().empty()) continue;
      auto fold=ecm.Component<gz::sim::components::JointPosition>(foldJoints[0]);
      const bool folded=fold && !fold->Data().empty() && foldTarget.load()>1.3 && std::abs(fold->Data()[0]-foldTarget.load())<.04;
      const double speed=velocity->Data()[0];
      const double desired=folded?direction[i]*wheelTarget[i].load():0.;
      // Coast while morphing: the linkage must not fight a locked wheel axle.
      const double effort=folded?std::clamp(.8*(desired-speed),-.33,.33):0.;
      auto force=ecm.Component<gz::sim::components::JointForceCmd>(wheels[i]);
      if(force) force->Data()={effort};
      else ecm.CreateComponent(wheels[i],gz::sim::components::JointForceCmd({effort}));
    }
    for(int i=0;i<4;++i) {
      auto linkEntity=model.LinkByName(ecm,"prop_"+corners[i]);
      if(linkEntity==gz::sim::kNullEntity) continue; // shared-drive comparison uses its original plugin
      if(propJoints[i]==gz::sim::kNullEntity) {
        propJoints[i]=model.JointByName(ecm,"spin_"+corners[i]);
        if(!ecm.Component<gz::sim::components::JointVelocity>(propJoints[i])) ecm.CreateComponent(propJoints[i],gz::sim::components::JointVelocity());
      }
      auto velocity=ecm.Component<gz::sim::components::JointVelocity>(propJoints[i]);
      if(!velocity || velocity->Data().empty()) continue;
      const double speed=velocity->Data()[0];
      const double drag=8.4e-6*.016*speed*std::abs(speed);
      const double target=direction[i]*propTarget[i].load();
      const double effort=SpeedTorque(speed,target,1.1e-4,std::abs(target)>std::abs(speed)?.0125:.025,8.4e-6*.016,.33);
      auto force=ecm.Component<gz::sim::components::JointForceCmd>(propJoints[i]);
      if(force) force->Data()={effort};
      else ecm.CreateComponent(propJoints[i],gz::sim::components::JointForceCmd({effort}));
      const auto pose=gz::sim::worldPose(linkEntity,ecm);
      gz::sim::Link(linkEntity).AddWorldWrench(ecm,
        pose.Rot().RotateVector({0,0,8.4e-6*speed*speed}),pose.Rot().RotateVector({0,0,-drag}));
    }
    const double now=std::chrono::duration<double>(info.simTime).count();
    const bool broken=latches[0].broken || latches[1].broken;
    if(now-lastFaultTime>=.05 || broken!=lastFault) {
      gz::msgs::Boolean fault;fault.set_data(broken);faultPublisher.Publish(fault);
      lastFaultTime=now;lastFault=broken;
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    if(baseLink!=gz::sim::kNullEntity && now-lastOdomTime>=.02) {
      gz::sim::Link base(baseLink);
      auto linear=base.WorldLinearVelocity(ecm), angular=base.WorldAngularVelocity(ecm);
      if(linear && angular && linear->IsFinite() && angular->IsFinite()) {
        const auto pose=gz::sim::worldPose(baseLink,ecm);
        gz::msgs::Odometry msg;
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
        msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
        auto frame=msg.mutable_header()->add_data();frame->set_key("frame_id");frame->add_value("odom");
        auto child=msg.mutable_header()->add_data();child->set_key("child_frame_id");child->add_value("base_link");
        gz::msgs::Set(msg.mutable_pose(),pose);
        gz::msgs::Set(msg.mutable_twist()->mutable_linear(),pose.Rot().RotateVectorReverse(*linear));
        gz::msgs::Set(msg.mutable_twist()->mutable_angular(),pose.Rot().RotateVectorReverse(*angular));
        odomPublisher.Publish(msg);lastOdomTime=now;
      }
    }
    if(contactAudit.is_open() && now-lastContact>=.02) {
      lastContact=now;
      std::set<std::pair<std::string,std::string>> seen;
      ecm.Each<gz::sim::components::ContactSensorData>([&](const gz::sim::Entity &,const gz::sim::components::ContactSensorData *data) {
        for(const auto &contact:data->Data().contact()) {
          auto a=contact.collision1().name(),b=contact.collision2().name();if(a>b) std::swap(a,b);
          if(!seen.insert({a,b}).second)continue;
          double depth=0.;for(double d:contact.depth())depth=std::max(depth,d);
          double rz=0.,az=0.;
          for(const auto &corner:{"fl","fr","rl","rr"}) if(b.find(std::string("ring_")+corner+"::")!=std::string::npos) {
            auto pose=gz::sim::worldPose(model.LinkByName(ecm,std::string("ring_")+corner),ecm);rz=pose.Pos().Z();az=pose.Rot().RotateVector(gz::math::Vector3d::UnitZ).Z();
          }
          contactAudit<<now<<','<<a<<','<<b<<','<<depth<<','<<(contact.position_size()?contact.position(0).z():0.)<<','<<rz<<','<<az<<','<<foldTarget.load()<<','<<(contact.wrench_size()?contact.wrench(0).body_1_wrench().force().z():0.)<<'\n';
        }return true;});
      contactAudit.flush();
    }
    if(!audit.is_open() || now-lastAudit<.004) return;
    lastAudit=now;
    for(int i=0;i<2;++i) {
      auto q=ecm.Component<gz::sim::components::JointPosition>(steering[i]);
      auto v=ecm.Component<gz::sim::components::JointVelocity>(steering[i]);
      auto f=ecm.Component<gz::sim::components::JointPosition>(hinges[i]);
      auto w=ecm.Component<gz::sim::components::JointTransmittedWrench>(steering[i]);
      if(!q || q->Data().empty() || !v || v->Data().empty() || !f || f->Data().empty() || !w) continue;
      const auto &force=w->Data().force(); const auto &torque=w->Data().torque();
      audit<<now<<','<<i<<','<<gz::sim::worldPose(model.LinkByName(ecm,"base_link"),ecm).Pos().Z()<<','<<f->Data()[0]<<','<<q->Data()[0]<<','<<v->Data()[0]<<','
           <<force.x()<<','<<force.y()<<','<<force.z()<<','<<torque.x()<<','<<torque.y()<<','<<torque.z()<<','<<latchTorque[i]<<','<<latches[i].broken<<','<<(ecm.Component<gz::sim::components::JointVelocity>(hinges[i]) && !ecm.Component<gz::sim::components::JointVelocity>(hinges[i])->Data().empty()?ecm.Component<gz::sim::components::JointVelocity>(hinges[i])->Data()[0]:0.)<<'\n';
    }
    if(now-lastFlush>=1.) {audit.flush();lastFlush=now;}
  }
};
}
GZ_ADD_PLUGIN(ringquad::SteeringLock,gz::sim::System,
 ringquad::SteeringLock::ISystemConfigure,ringquad::SteeringLock::ISystemPreUpdate,ringquad::SteeringLock::ISystemPostUpdate)
