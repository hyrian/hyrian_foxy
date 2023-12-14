#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

class IsKeyword2EqualTo : public BT::ConditionNode
{
public:
    IsKeyword2EqualTo(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), nh(std::make_shared<rclcpp::Node>("IsKeyword2EqualTo"))
    {
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_pub", 10, std::bind(&IsKeyword2EqualTo::keywordCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("value") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyword_sub;
    std::string keyword2;

    void keywordCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream ss(msg->data);
        std::string tmp;
        std::getline(ss, tmp, ',');
        std::getline(ss, keyword2, ',');
    }
};

BT::NodeStatus IsKeyword2EqualTo::tick()
{
    BT::Optional<std::string> value = getInput<std::string>("value");

    if (!value) {
        return BT::NodeStatus::FAILURE;
    }

    return (keyword2 == *value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
