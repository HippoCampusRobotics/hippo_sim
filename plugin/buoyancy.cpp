// https://github.com/gazebosim/gz-sim/tree/ign-gazebo6/examples/plugin/system_plugin
#include "buoyancy.h"

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/plugin/Register.hh>

IGNITION_ADD_PLUGIN(buoyancy::BuoyancyPlugin, ignition::gazebo::System,
                    buoyancy::BuoyancyPlugin::ISystemConfigure,
                    buoyancy::BuoyancyPlugin::ISystemUpdate)
IGNITION_ADD_PLUGIN_ALIAS(buoyancy::BuoyancyPlugin,
                          "ignition::gazebo::systems::BuoyancyPlugin")

using namespace buoyancy;

BuoyancyPlugin::BuoyancyPlugin() : System() {}

BuoyancyPlugin::~BuoyancyPlugin() {}

void BuoyancyPlugin::Configure(const ignition::gazebo::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               ignition::gazebo::EntityComponentManager &_ecm,
                               ignition::gazebo::EventManager &_eventMgr) {
  entity_ = _entity;
  model_ = ignition::gazebo::Model(entity_);
  // ignmsg << "Buoyancy Plugin for entity [" << entity_ << "]" << std::endl;
  if (!model_.Valid(_ecm)) {
    ignerr << "Buoyancy Plugin can only be attached to a model entity."
           << std::endl;
    return;
  }

  auto sdf_clone = _sdf->Clone();
  for (auto element = sdf_clone->GetFirstElement(); element != nullptr;
       element = element->GetNextElement()) {
    buoyancy_s buoyancy;
    buoyancy.model = model_;
    // ignmsg << "Parsing element " << element->GetName() << std::endl;
    if (element->GetName() != "buoyancy") {
      continue;
    }
    ignmsg << "Found buoyancy element." << std::endl;
    ParseBuoyancyElement(element, buoyancy, _ecm);
    buoyancy_links_.push_back(buoyancy);
  }
}

void BuoyancyPlugin::ParseBuoyancyElement(
    const sdf::ElementPtr _element, buoyancy_s &buoyancy,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  double force_added = 0.0;
  double compensation = 0.0;
  if (_element->HasElement("link_name")) {
    buoyancy.link_name = _element->Get<std::string>("link_name");
  } else {
    ignerr << "No link_name provided in sdf." << std::endl;
    return;
  }
  buoyancy.link =
      ignition::gazebo::Link(model_.LinkByName(_ecm, buoyancy.link_name));

  if (_element->HasElement("force_added")) {
    force_added = _element->Get<double>("force_added");
  } else {
    ignerr << "Could not find force_added element." << std::endl;
  }

  if (_element->HasElement("compensation")) {
    compensation = _element->Get<double>("compensation");
  } else {
    ignerr << "Could not find compensation element." << std::endl;
  }
  buoyancy.compensation = compensation;
  buoyancy.force_added = ignition::math::Vector3d(0.0, 0.0, force_added);

  if (_element->HasElement("origin")) {
    buoyancy.center_of_buoyancy =
        _element->Get<ignition::math::Vector3d>("origin");
  } else {
    ignerr << "Could not find origin element." << std::endl;
  }

  if (_element->HasElement("height_scale_limit")) {
    buoyancy.height_scale_limit = _element->Get<double>("height_scale_limit");
  } else {
    ignerr << "Could not find height_scale_limit element." << std::endl;
  }
}

void BuoyancyPlugin::Update(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  // get the world if not already done
  if (world_.Entity() == ignition::gazebo::kNullEntity) {
    world_entity_ = ignition::gazebo::worldEntity(entity_, _ecm);
    world_ = ignition::gazebo::World(world_entity_);
  }
  if (!world_.Valid(_ecm)) {
    ignerr << "Could not identify world" << std::endl;
    return;
  }
  for (auto &buoyancy : buoyancy_links_) {
    if (buoyancy.link.Entity() == ignition::gazebo::kNullEntity) {
      buoyancy.link =
          ignition::gazebo::Link(model_.LinkByName(_ecm, buoyancy.link_name));
    }
    if (!buoyancy.link.Valid(_ecm)) {
      // ignerr << "Could not identify link [" << buoyancy.link_name << "]"
      //        << std::endl;
      continue;
    }
    auto inertial = _ecm.Component<ignition::gazebo::components::Inertial>(
        buoyancy.link.Entity());
    auto mass = inertial->Data().MassMatrix().Mass();
    ignition::math::Pose3d pose = ignition::gazebo::worldPose(entity_, _ecm);
    ignition::math::Vector3d torque_offset =
        pose.Rot().RotateVector(buoyancy.center_of_buoyancy);
    ignition::math::Vector3d center_of_buoyancy =
        pose.Pos() + pose.Rot().RotateVector(buoyancy.center_of_buoyancy);
    // TODO: replace gravity numeric value with world gravity
    ignition::math::Vector3d force =
        buoyancy.force_added +
        ignition::math::Vector3d(0.0, 0.0, 9.81 * mass * buoyancy.compensation);
    double scale =
        std::abs((center_of_buoyancy.Z() - buoyancy.height_scale_limit) /
                 (2 * buoyancy.height_scale_limit));
    if (center_of_buoyancy.Z() > buoyancy.height_scale_limit) {
      scale = 0.0;
    }
    scale = ignition::math::clamp(scale, 0.0, 1.0);
    force *= scale;
    auto torque = force.Cross(torque_offset);
    buoyancy.link.AddWorldWrench(_ecm, force, torque);
  }
}
