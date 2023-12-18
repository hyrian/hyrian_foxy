#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp_v3/action_node.h>

class KeywordTrigger : public BT::ActionNodeBase
{
public:
    KeywordTrigger(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), nh(std::make_shared<rclcpp::Node>("KeywordTrigger"))
    {
        keyword_sub = nh->create_subscription<std_msgs::msg::String>(
            "keyword_topic", 10, std::bind(&KeywordTrigger::keywordCallback, this, std::placeholders::_1));

        spin_thread = std::make_shared<std::thread>([this]() {
            rclcpp::spin(this->nh);
        });
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<std::string>("keyword1"),
                BT::OutputPort<std::string>("keyword2") };
    }


    BT::NodeStatus tick() override
    {
        if (keyword1.empty() || keyword2.empty()) {
            return BT::NodeStatus::RUNNING;
        }

        setResult(keyword1, keyword2);
        RCLCPP_INFO(nh->get_logger(), "KeywordCheck: keyword1 = %s, keyword2 = %s", keyword1.c_str(), keyword2.c_str());

        keyword1.clear();
        keyword2.clear();
        return BT::NodeStatus::SUCCESS;
    }

    void halt() override
    {
        if (spin_thread->joinable()) {
            spin_thread->join();
        }
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
        setResult(keyword1, keyword2);
    }

    void setResult(const std::string& keyword1, const std::string& keyword2)
    {
        setOutput("keyword1", keyword1);
        setOutput("keyword2", keyword2);
    }

    void clean_up()
    {
        rclcpp::shutdown();
        if (spin_thread->joinable()) {
            spin_thread->join();
        }
    }
};




