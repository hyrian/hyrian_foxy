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
        BT::Optional<std::string> opt_value = getInput<std::string>("value");
        BT::Optional<std::string> opt_keyword1 = getInput<std::string>("keyword");

        if (!opt_value || !opt_keyword1) {
            std::cout << "Waiting for message..." << std::endl;
            return BT::NodeStatus::RUNNING;
        }

        std::string value = opt_value.value();
        std::string keyword = opt_keyword1.value();

        RCLCPP_INFO(nh->get_logger(), "KeywordCheck: value = %s, keyword = %s", value.c_str(), keyword.c_str());

        if (value == keyword) {
            // setOutput("keyword2_out", keyword2);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        return{ 
                BT::InputPort<std::string>("value"),
                BT::InputPort<std::string>("keyword"),
                // BT::OutputPort<std::string>("keyword2_out") 
                };
    }

private:
    rclcpp::Node::SharedPtr nh;
    std::shared_ptr<std::thread> spin_thread;
};