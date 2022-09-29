#include "odometry.hpp"

#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/pose.pb.h>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#define SDF_MISSING_ELEMENT(x) \
  (ignerr << "Could not find [" << x << "] element in sdf." << std::endl)

IGNITION_ADD_PLUGIN(odometry::OdometryPlugin, ignition::gazebo::System,
                    odometry::OdometryPlugin::ISystemConfigure,
                    odometry::OdometryPlugin::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(odometry::OdometryPlugin,
                          "ignition::gazebo::systems::OdometryPlugin")

using namespace ignition;
using namespace gazebo;

namespace odometry {
class OdometryPluginPrivate {
 public:
  std::chrono::steady_clock::duration update_period_{0};
  std::chrono::steady_clock::duration last_pub_time_{0};

  void ParseSdf(const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm) {
    sdf_params_.link = _sdf->Get<std::string>("link", sdf_params_.link).first;
    sdf_params_.base_topic =
        _sdf->Get<std::string>("base_topic", sdf_params_.base_topic).first;
    sdf_params_.update_rate =
        _sdf->Get<double>("update_rate", sdf_params_.update_rate).first;
    if (sdf_params_.update_rate > 0.0) {
      std::chrono::duration<double> period{1.0 / sdf_params_.update_rate};
      update_period_ =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              period);
    }
  }

  bool InitModel(EntityComponentManager &_ecm, Entity _entity) {
    model_ = Model(_entity);
    if (!model_.Valid(_ecm)) {
      return false;
    }
    SetModelName(model_.Name(_ecm));
    InitEntities(_ecm);
    InitPoseHeader();
    return true;
  }

  void PublishPose(const EntityComponentManager &_ecm,
                   const msgs::Time &stamp) {
    auto pose = link_.WorldPose(_ecm);
    auto v_linear = link_.WorldLinearVelocity(_ecm);
    auto v_angular = link_.WorldAngularVelocity(_ecm);
    auto v_angular_local = pose->Rot().Inverse().RotateVector(*v_angular);
    auto header = odometry_msg_.mutable_header();
    auto twist = odometry_msg_.mutable_twist();
    header->mutable_stamp()->CopyFrom(stamp);
    msgs::Set(odometry_msg_.mutable_pose(), *pose);
    msgs::Set(twist->mutable_angular(), v_angular_local);
    msgs::Set(twist->mutable_linear(), *v_linear);
    odometry_pub_.Publish(odometry_msg_);
  }

  void Advertise() {
    odometry_pub_ = node_.Advertise<msgs::Odometry>(TopicName());
  }

  const std::string TopicName() {
    return "/" + model_name_ + "/" + sdf_params_.base_topic;
  }

 private:
  struct SdfParams {
    std::string link{"base_link"};
    double update_rate{10.0};
    std::string base_topic{"odometry"};
  } sdf_params_;

  void InitPoseHeader() {
    auto header = odometry_msg_.mutable_header();
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value("map");
  }

  void SetModel(Entity _entity) { ; }

  void SetModelName(const std::string _name) { model_name_ = _name; }

  std::string ModelName() { return model_name_; }

  std::string LinkName() { return sdf_params_.link; }

  void InitEntities(EntityComponentManager &_ecm) {
    link_ = Link(model_.LinkByName(_ecm, sdf_params_.link));
    if (!_ecm.Component<components::WorldPose>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), components::WorldPose());
    }
    if (!_ecm.Component<components::AngularVelocity>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), components::AngularVelocity());
    }
    if (!_ecm.Component<components::LinearVelocity>(link_.Entity())) {
      _ecm.CreateComponent(link_.Entity(), components::LinearVelocity());
    }
  }

  ignition::gazebo::Model model_{kNullEntity};
  std::string model_name_ = "unknown_model_name";
  Link link_{kNullEntity};
  transport::Node node_;
  transport::Node::Publisher odometry_pub_;
  msgs::Odometry odometry_msg_;
};

OdometryPlugin::OdometryPlugin()
    : System(), private_(std::make_unique<OdometryPluginPrivate>()) {}

void OdometryPlugin::Configure(const ignition::gazebo::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               ignition::gazebo::EntityComponentManager &_ecm,
                               ignition::gazebo::EventManager &_eventMgr) {
  private_->ParseSdf(_sdf, _ecm);
  if (!private_->InitModel(_ecm, _entity)) {
    ignerr << "OdometryPlugin needs to be attached to model entity."
           << std::endl;
    return;
  }
  private_->Advertise();
}
void OdometryPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) {
  if (_info.paused) {
    return;
  }

  auto dt = _info.simTime - private_->last_pub_time_;
  if ((dt > std::chrono::steady_clock::duration::zero()) &&
      (dt < private_->update_period_)) {
    return;
  }

  private_->last_pub_time_ = _info.simTime;
  private_->PublishPose(_ecm, convert<msgs::Time>(_info.simTime));
}
}  // namespace odometry
