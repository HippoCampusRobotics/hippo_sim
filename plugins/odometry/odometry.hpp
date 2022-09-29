#pragma once
#include <ignition/gazebo/System.hh>

namespace odometry {
class OdometryPluginPrivate;
class OdometryPlugin : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemConfigure,
                       public ignition::gazebo::ISystemPostUpdate {
 public:
  OdometryPlugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<OdometryPluginPrivate> private_;
};
}  // namespace odometry
