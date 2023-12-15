#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hyrian_interfaces/srv/get_gesture.hpp"

class Gesture : public BT::SyncActionNode
{
public:
    Gesture(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh(std::make_shared<rclcpp::Node>("Gesture"))
    {
        client = nh->create_client<hyrian_interfaces::srv::GetGesture>("gesture_service");
    }

    ~Gesture()
    {
        rclcpp::shutdown();
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<std::string> keyword2_ = getInput<std::string>("option");
        

        if (!keyword2_) {
            RCLCPP_ERROR(nh->get_logger(), "No keyword2 received");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(nh->get_logger(), "Received keyword2: %s", keyword2_->c_str());

        auto request = std::make_shared<hyrian_interfaces::srv::GetGesture::Request>();
        request->gesture = *keyword2_;

        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(nh, future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.valid()) {
                auto response = future.get();
                RCLCPP_INFO(nh->get_logger(), "Received response: %s", response->finish.c_str());
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(nh->get_logger(), "Future is not ready");
                return BT::NodeStatus::FAILURE;
            }
        } else {
            RCLCPP_ERROR(nh->get_logger(), "Service call failed");
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("option") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Client<hyrian_interfaces::srv::GetGesture>::SharedPtr client;
};