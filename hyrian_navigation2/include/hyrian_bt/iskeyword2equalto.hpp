#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

class IsKeyword2EqualTo : public BT::ConditionNode
{
public:
    IsKeyword2EqualTo(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) { }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("value"), BT::InputPort<std::string>("keyword2") };
    }
};

BT::NodeStatus IsKeyword2EqualTo::tick()
{
    BT::Optional<std::string> value = getInput<std::string>("value");
    BT::Optional<std::string> keyword2 = getInput<std::string>("keyword2");

    if (!value || !keyword2) {
        return BT::NodeStatus::FAILURE;
    }

    return (*keyword2 == *value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}



