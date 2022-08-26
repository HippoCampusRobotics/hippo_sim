// https://github.com/gazebosim/gz-sim/tree/ign-gazebo6/examples/plugin/system_plugin
#pragma once

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/math.hh>

namespace buoyancy {
class BuoyancyPlugin : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemConfigure,
                       public ignition::gazebo::ISystemUpdate {
 public:
  BuoyancyPlugin();

  ~BuoyancyPlugin() override;
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void Update(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  struct buoyancy_s {
    ignition::gazebo::Model model;
    std::string link_name;
    ignition::gazebo::Link link;
    double compensation;
    ignition::math::Vector3d force_added;
    ignition::math::Vector3d center_of_buoyancy;
    double height_scale_limit;
  };
  void ParseBuoyancyElement(
      const sdf::ElementPtr _element, buoyancy_s &buyoancy,
      const ignition::gazebo::EntityComponentManager &_ecm);
  std::vector<buoyancy_s> buoyancy_links_;
  ignition::gazebo::Entity entity_;
  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::gazebo::World world_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity world_entity_;
};
}  // namespace buoyancy
