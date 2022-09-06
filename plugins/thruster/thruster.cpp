#include "thruster.h"

#include <ignition/msgs/double.pb.h>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/ParentLinkName.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

IGNITION_ADD_PLUGIN(thruster::ThrusterPlugin, ignition::gazebo::System,
                    thruster::ThrusterPlugin::ISystemConfigure,
                    thruster::ThrusterPlugin::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(thruster::ThrusterPlugin,
                          "ignition::gazebo::systems::ThrusterPlugin")

using namespace thruster;
using namespace ignition;
using namespace gazebo;

namespace turning_direction {
const static int CCW = -1;
const static int CW = 1;
};  // namespace turning_direction

class thruster::ThrusterPluginData {
 public:
  ignition::gazebo::Model model_{kNullEntity};
  Link link_{kNullEntity};
  Link parent_link_{kNullEntity};
  Entity joint_entity_{kNullEntity};

  transport::Node node;

  double linear_coeff{0.0};
  double quadratic_coeff{0.0};
  double torque_constant{0.0};
  double rpm_scaler{10.0};
  double max_rpm{100.0};
  int turning_direction{turning_direction::CCW};
  int propeller_direction{turning_direction::CCW};

  int thruster_number;

  std::mutex thrust_cmd_mutex;
  double thrust_cmd;

  void OnThrustCmd(const ignition::msgs::Double &_msg) {
    std::lock_guard<std::mutex> lock(thrust_cmd_mutex);
    thrust_cmd = _msg.data();
  }

  void ApplyThrustAndTorque(EntityComponentManager &_ecm) {
    auto pose = link_.WorldPose(_ecm);
    auto parent_pose = parent_link_.WorldPose(_ecm);
    auto pose_diff = *pose - *parent_pose;
    link_.AddWorldForce(_ecm, pose->Rot().RotateVector(Thrust()));
    auto parent_wrench_component =
        _ecm.Component<components::ExternalWorldWrenchCmd>(
            parent_link_.Entity());
    auto parent_torque = pose_diff.Rot().RotateVector(Torque());
    auto parent_torque_world = parent_pose->Rot().RotateVector(parent_torque);
    if (parent_wrench_component) {
      msgs::Set(parent_wrench_component->Data().mutable_torque(),
                msgs::Convert(parent_wrench_component->Data().torque()) +
                    parent_torque_world);
    }
  }

  void UpdateSpeed(EntityComponentManager &_ecm) {
    auto velocity_component =
        _ecm.Component<components::JointVelocityCmd>(joint_entity_);
    if (!velocity_component) {
      CreateJointComponents(_ecm, joint_entity_);
    } else if (!velocity_component->Data().empty()) {
      velocity_component->Data()[0] = Speed();
    }
  }

  void CreateJointComponents(ignition::gazebo::EntityComponentManager &_ecm,
                             ignition::gazebo::Entity _joint) {
    if (!_ecm.EntityHasComponentType(_joint,
                                     components::JointVelocity().TypeId())) {
      _ecm.CreateComponent(joint_entity_, components::JointVelocity({0.0}));
    }

    if (!_ecm.EntityHasComponentType(_joint,
                                     components::JointVelocityCmd().TypeId())) {
      _ecm.CreateComponent(joint_entity_, components::JointVelocityCmd({0.0}));
    }
  }

  void CreateLinkComponents(ignition::gazebo::EntityComponentManager &_ecm) {
    if (!_ecm.Component<components::WorldPose>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), components::WorldPose());
    }
  }

  void CreateParentLinkComponents(
      ignition::gazebo::EntityComponentManager &_ecm) {
    if (!_ecm.Component<components::WorldPose>(parent_link_.Entity())) {
      _ecm.CreateComponent(parent_link_.Entity(), components::WorldPose());
    }

    if (!_ecm.Component<components::ExternalWorldWrenchCmd>(
            parent_link_.Entity())) {
      _ecm.CreateComponent(parent_link_.Entity(),
                           components::ExternalWorldWrenchCmd());
    }
  }

 private:
  math::Vector3d Thrust() {
    double input;
    {
      std::lock_guard<std::mutex> lock(thrust_cmd_mutex);
      input = thrust_cmd;
    }
    double thrust;
    double tmp = std::abs(input);
    thrust = tmp * tmp * quadratic_coeff + tmp * linear_coeff;
    if (input < 0) {
      thrust *= -1.0;
    }
    double force = turning_direction * propeller_direction * thrust;
    return math::Vector3d(force, 0, 0);
  }

  math::Vector3d Torque() {
    return -turning_direction * propeller_direction * Thrust() *
           torque_constant;
  }

  double Speed() {
    double input;
    {
      std::lock_guard<std::mutex> lock(thrust_cmd_mutex);
      input = thrust_cmd;
    }
    return input * turning_direction * max_rpm / 60.0 * 3.14 / rpm_scaler;
  }
};

ThrusterPlugin::ThrusterPlugin()
    : System(), data_(std::make_unique<ThrusterPluginData>()) {}

ThrusterPlugin::~ThrusterPlugin() {}

void ThrusterPlugin::Configure(const ignition::gazebo::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               ignition::gazebo::EntityComponentManager &_ecm,
                               ignition::gazebo::EventManager &_eventMgr) {
  data_->model_ = ignition::gazebo::Model(_entity);
  ParseSdf(_sdf, _ecm);
  const auto parent_name =
      _ecm.Component<components::ParentLinkName>(data_->joint_entity_)->Data();
  const auto parent_entity = data_->model_.LinkByName(_ecm, parent_name);
  data_->parent_link_ = Link(parent_entity);

  data_->CreateJointComponents(_ecm, data_->joint_entity_);
  data_->CreateLinkComponents(_ecm);
  data_->CreateParentLinkComponents(_ecm);

  std::string topic_name = transport::TopicUtils::AsValidTopic(
      "/" + data_->model_.Name(_ecm) + "/thruster_" +
      std::to_string(data_->thruster_number) + "/thrust");
  data_->node.Subscribe(topic_name, &ThrusterPluginData::OnThrustCmd,
                        data_.get());
}

void ThrusterPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  data_->ApplyThrustAndTorque(_ecm);
  data_->UpdateSpeed(_ecm);
}

void ThrusterPlugin::ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                              ignition::gazebo::EntityComponentManager &_ecm) {
  for (sdf::ElementPtr element = _sdf->GetFirstElement(); element != nullptr;
       element = element->GetNextElement()) {
    if (element->GetName() != "thruster_config") {
      continue;
    }
    ignmsg << "Found thruster element." << std::endl;
    ParseThrusterParams(element, _ecm);
  }
}

void ThrusterPlugin::ParseThrusterParams(
    const sdf::ElementPtr _element,
    ignition::gazebo::EntityComponentManager &_ecm) {
  std::string name;

  name = "link";
  if (_element->HasElement(name)) {
    std::string link_name = _element->Get<std::string>(name);
    data_->link_ = Link(data_->model_.LinkByName(_ecm, link_name));
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "joint";
  if (_element->HasElement(name)) {
    std::string joint_name = _element->Get<std::string>(name);
    data_->joint_entity_ = data_->model_.JointByName(_ecm, joint_name);
    if (data_->joint_entity_ != kNullEntity) {
    } else {
      ignerr << "Joint with name [" << joint_name << "] not found!"
             << std::endl;
    }
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "linear_coeff";
  if (_element->HasElement(name)) {
    data_->linear_coeff = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "quadratic_coeff";
  if (_element->HasElement(name)) {
    data_->quadratic_coeff = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "torque_coeff";
  if (_element->HasElement(name)) {
    data_->torque_constant = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "rpm_scaler";
  if (_element->HasElement(name)) {
    data_->rpm_scaler = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "max_rpm";
  if (_element->HasElement(name)) {
    data_->max_rpm = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "thruster_number";
  if (_element->HasElement(name)) {
    data_->thruster_number = _element->Get<int>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "turning_direction";
  if (_element->HasElement(name)) {
    std::string direction = _element->Get<std::string>(name);
    if (direction == "cw") {
      data_->turning_direction = turning_direction::CW;
    } else {
      data_->turning_direction = turning_direction::CCW;
    }
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "propeller_direction";
  if (_element->HasElement(name)) {
    std::string direction = _element->Get<std::string>(name);
    if (direction == "cw") {
      data_->propeller_direction = turning_direction::CW;
    } else {
      data_->propeller_direction = turning_direction::CCW;
    }
  } else {
    SDF_MISSING_ELEMENT(name);
  }
}
