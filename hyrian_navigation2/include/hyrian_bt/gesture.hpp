#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <dynamixel_sdk_custom_interfaces/srv/get_gesture.hpp> // replace with your service header

class Gesture : public BT::SyncActionNode
{
public:
    Gesture(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh(std::make_shared<rclcpp::Node>("Gesture"))
    {
        client = nh->create_client<dynamixel_sdk_custom_interfaces::srv::GetGesture>("gesture_service");
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("value") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetGesture>::SharedPtr client;

    virtual BT::NodeStatus tick() override
    {
        BT::Optional<std::string> value = getInput<std::string>("value");

        if (!value) {
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetGesture::Request>();
        request->gesture = *value;

        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(nh, future) == rclcpp::FutureReturnCode::SUCCESS) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
};
