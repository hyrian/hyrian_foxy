#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/condition_node.h>

class KeywordCheck : public BT::ConditionNode
{
public:
    KeywordCheck(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), nh(std::make_shared<rclcpp::Node>("KeywordCheck"))
    {
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_pub", 10, std::bind(&KeywordCheck::keywordCallback, this, std::placeholders::_1));

        spin_thread = std::make_shared<std::thread>([this]() {
            while (rclcpp::ok() && keyword2.empty()) {
                rclcpp::spin_some(this->nh);
            }
        });
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
    std::string keyword2;
    std::shared_ptr<std::thread> spin_thread;

    void keywordCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream ss(msg->data);
        std::getline(ss, keyword1, ',');
        std::getline(ss, keyword2, ',');
        setOutput("keyword2", keyword2);
        RCLCPP_INFO(nh->get_logger(), "Received message: %s", msg->data.c_str());
        RCLCPP_INFO(nh->get_logger(), "Keyword2: %s", keyword2.c_str()); 
    }
};

BT::NodeStatus KeywordCheck::tick()
{
    BT::Optional<std::string> value = getInput<std::string>("value");

    if (!value || keyword1.empty()) {
        RCLCPP_INFO(nh->get_logger(), "Waiting for message...");
        return BT::NodeStatus::RUNNING;
    }
    // if (!value) {
    //     return BT::NodeStatus::FAILURE;
    // }
    RCLCPP_INFO(nh->get_logger(), "Received input value: %s", value->c_str());
    
    return (keyword1 == *value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}