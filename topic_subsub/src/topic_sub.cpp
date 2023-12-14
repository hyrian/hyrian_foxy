#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscriber : public rclcpp::Node {
public:
  TopicSubscriber()
      : Node("topic_subscriber") {
    subscription_person_ = this->create_subscription<std_msgs::msg::String>(
        "person", 10, std::bind(&TopicSubscriber::person_callback, this, std::placeholders::_1));

    subscription_stop_ = this->create_subscription<std_msgs::msg::String>(
        "stop", 10, std::bind(&TopicSubscriber::stop_callback, this, std::placeholders::_1));
  }

private:
  void person_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received person message: %s", msg->data.c_str());
  }

  void stop_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received stop message: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_person_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_stop_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
