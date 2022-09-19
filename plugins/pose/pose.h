#pragma once
#include <ignition/gazebo/System.hh>

namespace pose {
class PosePluginData;
class PosePlugin : public ignition::gazebo::System,
                   public ignition::gazebo::ISystemConfigure,
                   public ignition::gazebo::ISystemPostUpdate {
 public:
  PosePlugin();
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<PosePluginData> data_;
};
}  // namespace pose
