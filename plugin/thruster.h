#pragma once
#include <ignition/gazebo/System.hh>

namespace thruster {
class ThrusterPluginData;
class ThrusterPlugin : public ignition::gazebo::System,
                       public ignition::gazebo::ISystemConfigure,
                       public ignition::gazebo::ISystemPreUpdate {
 public:
  ThrusterPlugin();
  ~ThrusterPlugin() override;
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

 private:
  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                ignition::gazebo::EntityComponentManager &_ecm);

  void ParseThrusterParams(const sdf::ElementPtr _element,
                           ignition::gazebo::EntityComponentManager &_ecm);

  void CreateJointComponents(ignition::gazebo::EntityComponentManager &_ecm,
                             ignition::gazebo::Entity _joint);

  std::unique_ptr<ThrusterPluginData> data_;
};
}  // namespace thruster
