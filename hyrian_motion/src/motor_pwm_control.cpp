#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motor_pwm_control/msg/set_motor_pwm.hpp"
#include "motor_pwm_control/msg/motor_pwm.hpp"

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode()
    : Node("motor_control_node")
  {
    portHandler = dynamixel_sdk::PortHandler::getPortHandler("/dev/ttyUSB0"); // 포트 설정
    packetHandler = dynamixel_sdk::PacketHandler::getPacketHandler(1.0);


    motor_id = 1; // 세팅
    
    publisher_ = this->create_publisher<motor_pwm_control::msg::MotorPWM>("motor_pwm", 10);

    subscription_ = this->create_subscription<motor_pwm_control::msg::SetMotorPWM>(
      "set_motor_pwm", 10, std::bind(&MotorControlNode::set_pwm_callback, this, std::placeholders::_1));
  }

private:
  void set_pwm_callback(const motor_pwm_control::msg::SetMotorPWM::SharedPtr msg)
  {
    int pwm_value = msg->pwm_value;

    int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, motor_id, 64, pwm_value);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send PWM to motor");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Set PWM to %d", pwm_value);

      auto pwm_msg = motor_pwm_control::msg::MotorPWM();
      pwm_msg.pwm_value = pwm_value;
      publisher_->publish(pwm_msg);
    }
  }

  rclcpp::Publisher<motor_pwm_control::msg::MotorPWM>::SharedPtr publisher_;
  rclcpp::Subscription<motor_pwm_control::msg::SetMotorPWM>::SharedPtr subscription_;
  dynamixel_sdk::PortHandler *portHandler;
  dynamixel_sdk::PacketHandler *packetHandler;
  int motor_id;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}
