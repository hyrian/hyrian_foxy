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
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_pub", 10, std::bind(&Gesture::keywordCallback, this, std::placeholders::_1));
        spin_thread = std::make_shared<std::thread>([this]() {
            while (rclcpp::ok() && keyword2.empty()) {
                rclcpp::spin_some(this->nh);
            }
        });
    }

    ~Gesture()
    {
        rclcpp::shutdown();
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("keyword2") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Client<hyrian_interfaces::srv::GetGesture>::SharedPtr client;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyword_sub;
    std::string keyword1;
    std::string keyword2;
    std::shared_ptr<std::thread> spin_thread;

    void keywordCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream ss(msg->data);
        std::getline(ss, keyword1, ',');
        std::getline(ss, keyword2, ',');
    }

    virtual BT::NodeStatus tick() override
    {
        try {
            if (keyword2.empty()) {
                RCLCPP_ERROR(nh->get_logger(), "No keyword2 received");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(nh->get_logger(), "Received keyword2: %s", keyword2.c_str());
            auto request = std::make_shared<hyrian_interfaces::srv::GetGesture::Request>();
            request->gesture = keyword2;
            RCLCPP_INFO(nh->get_logger(), "Sending request: %s", request->gesture.c_str());

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
        } catch (const std::exception& e) {
            RCLCPP_ERROR(nh->get_logger(), "Exception caught: %s", e.what());
            return BT::NodeStatus::FAILURE;
        }
    }
};