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

template <typename T>
class FirstOrderFilter {
 public:
  FirstOrderFilter(double _tau_up, double _tau_down, T _state)
      : tau_up_(_tau_up), tau_down_(_tau_down), state_(_state) {}
  T Update(T _state, double _dt) {
    T output;
    double alpha;
    if (_state > state_) {
      alpha = exp(-_dt / tau_up_);
    } else {
      alpha = exp(-_dt / tau_down_);
    }
    output = alpha * state_ + (1.0 - alpha) * _state;
    state_ = output;
    return output;
  }

 private:
  double tau_up_;
  double tau_down_;
  T state_;
};

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
  double timeconstant_up{0.0};
  double timeconstant_down{0.0};

  int thruster_number;

  std::mutex thrust_cmd_mutex;
  double thrust_cmd{0.0};
  double rotor_velocity_setpoint{0.0};
  double rotor_velocity{0.0};

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;

  void OnThrustCmd(const ignition::msgs::Double &_msg) {
    std::lock_guard<std::mutex> lock(thrust_cmd_mutex);
    thrust_cmd = _msg.data();
    rotor_velocity_setpoint = ThrustToVelocity(thrust_cmd);
  }

  double RotorVelocity(EntityComponentManager &_ecm) {
    auto velocity_component =
        _ecm.Component<components::JointVelocity>(joint_entity_);
    if (!velocity_component) {
      ignerr << "Joint has no velocity component!" << std::endl;
      return 0.0;
    } else if (!velocity_component->Data().empty()) {
      return velocity_component->Data()[0] * rpm_scaler;
    }
    ignerr << "Joint velocity data is empty!" << std::endl;
    return 0.0;
  }

  void SetRotorVelocity(EntityComponentManager &_ecm, double velocity) {
    auto velocity_component =
        _ecm.Component<components::JointVelocityCmd>(joint_entity_);
    if (!velocity_component) {
      CreateJointComponents(_ecm, joint_entity_);
    } else if (!velocity_component->Data().empty()) {
      velocity_component->Data()[0] = velocity / rpm_scaler;
    }
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

  double ThrustToVelocity(double thrust) {
    return thrust * turning_direction * max_rpm / 60.0 * 3.14;
  }

  void UpdateRotorVelocity(EntityComponentManager &_ecm, double dt) {
    {
      std::lock_guard<std::mutex> lock(thrust_cmd_mutex);
      rotor_velocity =
          rotor_velocity_filter_->Update(rotor_velocity_setpoint, dt);
    }
    SetRotorVelocity(_ecm, rotor_velocity);
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
    double thrust;
    double tmp = std::abs(rotor_velocity);
    thrust = tmp * tmp * quadratic_coeff + tmp * linear_coeff;
    if (rotor_velocity < 0) {
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

  data_->rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
      data_->timeconstant_up, data_->timeconstant_down,
      data_->rotor_velocity_setpoint);

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

  data_->UpdateRotorVelocity(_ecm, std::chrono::duration<double>(_info.dt).count());
  data_->ApplyThrustAndTorque(_ecm);
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

  name = "timeconstant_up";
  if (_element->HasElement(name)) {
    data_->timeconstant_up = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }

  name = "timeconstant_down";
  if (_element->HasElement(name)) {
    data_->timeconstant_down = _element->Get<double>(name);
  } else {
    SDF_MISSING_ELEMENT(name);
  }
}
