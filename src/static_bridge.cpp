#include <ignition/transport/Node.hh>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_ign_bridge/convert.hpp>

class Bridge {
 public:
  Bridge() {
    node_topics = (rclcpp::node_interfaces::NodeTopics *)ros_node_
                      ->get_node_topics_interface()
                      .get();
    std::string topic_name;
    rclcpp::SensorDataQoS sensor_qos;

    topic_name = node_topics->resolve_topic_name("imu");
    sensor_qos.keep_last(100);
    imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>(topic_name,
                                                                  sensor_qos);
    gz_node_->Subscribe(topic_name, &Bridge::OnImu, this);
    std::cout << node_topics->resolve_topic_name("imu") << std::endl;

    for (int i = 0; i < 4; i++) {
      topic_name = node_topics->resolve_topic_name("thruster_") +
                   std::to_string(i) + "/thrust";
      RCLCPP_INFO(ros_node_->get_logger(), "Create subscription: [%s]",
                  topic_name.c_str());
      thrust_pubs_[i] = gz_node_->Advertise<ignition::msgs::Double>(topic_name);
      std::function<void(const std_msgs::msg::Float64::SharedPtr _msg)> fn =
          std::bind(&Bridge::OnThrust, this, std::placeholders::_1, i);
      rclcpp::SensorDataQoS qos;
      qos.keep_last(50);
      thrust_subs_[i] =
      ros_node_->create_subscription<std_msgs::msg::Float64>(
          topic_name, qos, fn);
    }
  }

  void OnImu(const ignition::msgs::IMU &_msg) {
    sensor_msgs::msg::Imu ros_msg;
    ros_ign_bridge::convert_ign_to_ros(_msg, ros_msg);
    imu_pub_->publish(ros_msg);
  }

  void OnThrust(const std_msgs::msg::Float64::SharedPtr _msg, int i) {
    ignition::msgs::Double gz_msg;
    ros_ign_bridge::convert_ros_to_ign(*_msg, gz_msg);
    thrust_pubs_[i].Publish(gz_msg);
  }

  void Run() {
    rclcpp::spin(ros_node_);
    ignition::transport::waitForShutdown();
  }

 private:
  rclcpp::Node::SharedPtr ros_node_ = std::make_shared<rclcpp::Node>("bridge");
  std::shared_ptr<ignition::transport::Node> gz_node_ =
      std::make_shared<ignition::transport::Node>();
  rclcpp::node_interfaces::NodeTopics *node_topics;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::map<int, ignition::transport::Node::Publisher> thrust_pubs_;
  std::map<int, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr>
      thrust_subs_;
};

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto bridge = Bridge();
  bridge.Run();
}
