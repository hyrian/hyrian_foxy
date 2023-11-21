#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <chrono>

using namespace std::chrono_literals;
class MotorRotationNode : public rclcpp::Node
{
public:
  MotorRotationNode()
  : Node("motor_rotation_node")
  {
    publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&MotorRotationNode::timer_callback, this));
    id_ = 1;
    position_ = 0;
  }

private:
  void timer_callback()
  {
    auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
    message.id = id_;
    message.position = position_;
    publisher_->publish(message);

    // position을 0과 4000 사이에서 주기적으로 변경
    position_ = (position_ == 4000) ? 0 : 4000;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
  uint8_t id_;
  uint32_t position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorRotationNode>());
  rclcpp::shutdown();
  return 0;
}