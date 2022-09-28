#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_ign_bridge/convert.hpp>

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace ignition;
using namespace nav_msgs::msg;
namespace gz_msgs = ignition::msgs;

class Bridge {
 public:
  Bridge() {
    node_topics = (rclcpp::node_interfaces::NodeTopics *)ros_node_
                      ->get_node_topics_interface()
                      .get();

    CreateGroundTruthBridge();
    CreateImuBridge();
    CreateThrusterBridge();
  }

  void CreateGroundTruthBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("ground_truth/pose");
    pose_pub_ = ros_node_->create_publisher<PoseStamped>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnPose, this);
    RCLCPP_INFO(ros_node_->get_logger(), "Create gz subscription: [%s]",
                topic_name.c_str());

    topic_name = node_topics->resolve_topic_name("ground_truth/odometry");
    odometry_pub_ = ros_node_->create_publisher<Odometry>(topic_name, qos);
    gz_node_->Subscribe(topic_name, &Bridge::OnOdometry, this);
  }

  void CreateImuBridge() {
    std::string topic_name;
    rclcpp::SystemDefaultsQoS qos;
    qos.keep_last(100);

    // ros publisher
    topic_name = node_topics->resolve_topic_name("imu");
    imu_pub_ = ros_node_->create_publisher<Imu>(topic_name, qos);

    // gazebo subscriber
    gz_node_->Subscribe(topic_name, &Bridge::OnImu, this);
  }

  void CreateThrusterBridge() {
    for (int i = 0; i < 4; i++) {
      std::string topic_name;
      rclcpp::SystemDefaultsQoS qos;
      qos.keep_last(50);
      topic_name = node_topics->resolve_topic_name("thruster_") +
                   std::to_string(i) + "/thrust";

      // gazebo publisher
      thrust_pubs_[i] = gz_node_->Advertise<gz_msgs::Double>(topic_name);

      // ros subscriber
      std::function<void(const Float64::SharedPtr _msg)> fn =
          std::bind(&Bridge::OnThrust, this, std::placeholders::_1, i);
      thrust_subs_[i] =
          ros_node_->create_subscription<Float64>(topic_name, qos, fn);

      RCLCPP_INFO(ros_node_->get_logger(), "Create subscription: [%s]",
                  topic_name.c_str());
    }
  }

  void OnImu(const gz_msgs::IMU &_msg) {
    sensor_msgs::msg::Imu ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_msg, ros_msg);
    imu_pub_->publish(ros_msg);
  }

  void OnPose(const gz_msgs::Pose &_msg) {
    geometry_msgs::msg::PoseStamped ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_msg, ros_msg);
    pose_pub_->publish(ros_msg);
  }

  void OnOdometry(const gz_msgs::Odometry &_msg) {
    Odometry ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_msg, ros_msg);
    odometry_pub_->publish(ros_msg);
  }

  void OnThrust(const std_msgs::msg::Float64::SharedPtr _msg, int i) {
    gz_msgs::Double gz_msg;
    ros_ign_bridge::convert_ros_to_ign(*_msg, gz_msg);
    thrust_pubs_[i].Publish(gz_msg);
  }

  void Run() {
    rclcpp::spin(ros_node_);
    ignition::transport::waitForShutdown();
  }

 private:
  rclcpp::Node::SharedPtr ros_node_ = std::make_shared<rclcpp::Node>("bridge");
  std::shared_ptr<transport::Node> gz_node_ =
      std::make_shared<transport::Node>();
  rclcpp::node_interfaces::NodeTopics *node_topics;

  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  std::map<int, transport::Node::Publisher> thrust_pubs_;
  std::map<int, rclcpp::Subscription<Float64>::SharedPtr> thrust_subs_;
};

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto bridge = Bridge();
  bridge.Run();
}
