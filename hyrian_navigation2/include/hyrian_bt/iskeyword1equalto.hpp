#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

class IsKeyword1EqualTo : public BT::ConditionNode
{
public:
    IsKeyword1EqualTo(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), nh(std::make_shared<rclcpp::Node>("IsKeyword1EqualTo"))
    {
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_pub", 10, std::bind(&IsKeyword1EqualTo::keywordCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("value") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyword_sub;
    std::string keyword1;

    void keywordCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream ss(msg->data);
        std::getline(ss, keyword1, ',');
    }
};

BT::NodeStatus IsKeyword1EqualTo::tick()
{
    BT::Optional<std::string> value = getInput<std::string>("value");

    if (!value) {
        return BT::NodeStatus::FAILURE;
    }

    return (keyword1 == *value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
