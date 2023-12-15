#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/action_node.h>

class KeywordTrigger : public BT::SyncActionNode
{
public:
    KeywordTrigger(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh(std::make_shared<rclcpp::Node>("KeywordTrigger"))
    {
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_pub", 10, std::bind(&KeywordTrigger::keywordCallback, this, std::placeholders::_1));

        spin_thread = std::make_shared<std::thread>([this]() {
            rclcpp::spin(this->nh);
        });
    }

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<std::string>("keyword1"),
                BT::OutputPort<std::string>("keyword2") };
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
        RCLCPP_INFO(nh->get_logger(), "Received message: %s", msg->data.c_str());
        RCLCPP_INFO(nh->get_logger(), "Keyword1: %s, Keyword2: %s", keyword1.c_str(), keyword2.c_str()); 
    }
};

BT::NodeStatus KeywordTrigger::tick()
{
    setOutput("keyword1", keyword1);
    setOutput("keyword2", keyword2);
    
    return BT::NodeStatus::SUCCESS;
}
