#include "thruster.h"

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
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

class thruster::ThrusterPluginData {
 public:
  Model model_{kNullEntity};
  Link link_{kNullEntity};
  Entity joint_entity_{kNullEntity};

  transport::Node node_;

  double linear_coeff{0.0};
  double quadratic_coeff{0.0};
  double torque_constant{0.0};
  double rpm_scaler{10.0};
  double max_rpm{100.0};
};

ThrusterPlugin::ThrusterPlugin()
    : System(), data_(std::make_unique<ThrusterPluginData>()) {}

ThrusterPlugin::~ThrusterPlugin() {}

void ThrusterPlugin::Configure(const ignition::gazebo::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               ignition::gazebo::EntityComponentManager &_ecm,
                               ignition::gazebo::EventManager &_eventMgr) {
  data_->model_ = Model(_entity);
  ParseSdf(_sdf, _ecm);
}

void ThrusterPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                               ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }
  auto velocity =
      _ecm.Component<components::JointVelocityCmd>(data_->joint_entity_);
  if (velocity == nullptr) {
    CreateJointComponents(_ecm, data_->joint_entity_);
  } else if (!velocity->Data().empty()) {
    velocity->Data()[0] = 3.14;
  } else {
    ignerr << ":-(" << std::endl;
  }
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
      CreateJointComponents(_ecm, data_->joint_entity_);
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
}

void ThrusterPlugin::CreateJointComponents(
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::Entity _joint) {
  if (!_ecm.EntityHasComponentType(_joint,
                                   components::JointVelocity().TypeId())) {
    _ecm.CreateComponent(data_->joint_entity_,
                         components::JointVelocity({0.0}));
  }

  if (!_ecm.EntityHasComponentType(_joint,
                                   components::JointVelocityCmd().TypeId())) {
    _ecm.CreateComponent(data_->joint_entity_,
                         components::JointVelocityCmd({0.0}));
  }
}
