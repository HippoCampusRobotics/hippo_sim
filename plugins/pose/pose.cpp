#include "pose.h"

#include <ignition/msgs/pose.pb.h>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

IGNITION_ADD_PLUGIN(pose::PosePlugin, ignition::gazebo::System,
                    pose::PosePlugin::ISystemConfigure,
                    pose::PosePlugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(pose::PosePlugin,
                          "ignition::gazebo::systems::PosePlugin")

using namespace pose;
using namespace ignition;
using namespace gazebo;

class pose::PosePluginData {
 public:
  ignition::gazebo::Model model_{kNullEntity};
  Link link_{kNullEntity};
  transport::Node node_;
  std::string link_name_ = "base_link";
  double update_rate_ = 10.0;
  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};
  msgs::Pose pose_msg_;

  transport::Node::Publisher pose_pub_;

  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm) {
    link_name_ = _sdf->Get<std::string>("link", link_name_).first;
    update_rate_ = _sdf->Get<double>("update_rate", update_rate_).first;
    if (update_rate_ > 0.0) {
      std::chrono::duration<double> period{1.0 / update_rate_};
      update_period_ =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              period);
    }
  }

  void InitEntities(EntityComponentManager &_ecm) {
    link_ = Link(model_.LinkByName(_ecm, link_name_));
    if (!_ecm.Component<components::WorldPose>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), components::WorldPose());
    }
  }

  void PublishPose(const EntityComponentManager &_ecm,
                   const msgs::Time &stamp) {
    auto pose = link_.WorldPose(_ecm);
    pose_msg_.mutable_header()->mutable_stamp()->CopyFrom(stamp);
    pose_msg_.set_name(link_name_);
    msgs::Set(&pose_msg_, *pose);
    pose_pub_.Publish(pose_msg_);
  }
};

PosePlugin::PosePlugin()
    : System(), data_(std::make_unique<PosePluginData>()) {}

void PosePlugin::Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) {
  data_->model_ = Model(_entity);
  if (!data_->model_.Valid(_ecm)) {
    ignerr << "PosePlugin needs to be attached to model entity." << std::endl;
    return;
  }
  data_->ParseSdf(_sdf, _ecm);
  data_->InitEntities(_ecm);
  data_->pose_pub_ = data_->node_.Advertise<msgs::Pose>(
      "/" + data_->model_.Name(_ecm) + "/pose");
}
void PosePlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  auto dt = _info.simTime - data_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < data_->update_period_)) {
    return;
  }

  data_->last_pub_time_ = _info.simTime;
  data_->PublishPose(_ecm, convert<msgs::Time>(_info.simTime));
}
