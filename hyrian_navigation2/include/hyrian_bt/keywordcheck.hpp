#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

class KeywordCheck : public BT::ConditionNode
{
public:
    KeywordCheck(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), nh(std::make_shared<rclcpp::Node>("KeywordCheck"))
    {
        spin_thread = std::make_shared<std::thread>([this]() {
            rclcpp::spin(this->nh);
        });
    }

    BT::NodeStatus tick() override
    {
        BT::Optional<std::string> opt_keyword1 = getInput<std::string>("keyword1");
        BT::Optional<std::string> opt_keyword2 = getInput<std::string>("keyword2");

        if (!opt_keyword1 || !opt_keyword2) {
            std::cout << "Waiting for message..." << std::endl;
            return BT::NodeStatus::RUNNING;
        }

        std::string keyword1 = opt_keyword1.value();
        std::string keyword2 = opt_keyword2.value();

        RCLCPP_INFO(nh->get_logger(), "KeywordCheck: keyword1 = %s, keyword2 = %s", keyword1.c_str(), keyword2.c_str());

        setOutput("keyword2_out", keyword2);

        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("keyword1"),
                BT::InputPort<std::string>("keyword2"),
                BT::OutputPort<std::string>("keyword2_out") };
    }

private:
    rclcpp::Node::SharedPtr nh;
    std::shared_ptr<std::thread> spin_thread;
};