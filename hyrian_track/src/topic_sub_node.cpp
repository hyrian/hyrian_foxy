#include "rclcpp/rclcpp.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/object_count.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> // 추가된 부분
#include "std_msgs/msg/string.hpp"  // 추가된 부분

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber()
    : Node("topic_subscriber")
  {
    subscription_boxes_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      "darknet_ros/bounding_boxes", 10, std::bind(&TopicSubscriber::boxes_callback, this, std::placeholders::_1));
    subscription_count_ = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
      "darknet_ros/found_object", 10, std::bind(&TopicSubscriber::count_callback, this, std::placeholders::_1));
    subscription_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/depth/points", 10, std::bind(&TopicSubscriber::pcl_callback, this, std::placeholders::_1));
    
     // 추가된 부분: 'person'이 감지되면 'person' 토픽 발행
    publisher_person_ = this->create_publisher<std_msgs::msg::String>("person", 10);

     // 추가된 부분: Z 값이 0.7 이하로 감지되면 'stop' 토픽 발행
    publisher_stop_ = this->create_publisher<std_msgs::msg::String>("stop", 10);
    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
  }

private:
  void boxes_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) const
  {
    bool detected_person = false;
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pointcloud_msg, *cloud);

    for (const auto& boundingBox : msg->bounding_boxes)
    {
      if (boundingBox.class_id == "person")
      {
        detected_person = true;
        break;
      }
    }

    if (detected_person)
    {
      RCLCPP_INFO(this->get_logger(), "Detected a person: 1");
      float min_distance = std::numeric_limits<float>::max(); // 거리에 따른 속도 설정ㅇㅇ
      for (const auto& point : cloud->points)
      {
        float z = point.z;
        if(z < mid_dist){
          min_distance = z;
        }
        RCLCPP_INFO(this->get_logger(), "Distance: %f", z);
      }

      auto message = std_msgs::msg::String();
      message.data = "person";
      publisher_person_->publish(message);

      // pub cmv_vel , linvel angvel
      auto cmd_vel_msg = geometry_msgs::msg::Twist();
      cmd_vel_msg.linear.x = 0.2;
      cmd_vel_msg.linear.x = 0.3;
      cmd_vel.msg.angular.z = 0.0;

      // // 선속도 제어 - 거리에 따라
      // if(min_distance > 0.7)
      // {
      //   cmd_vel_msg.linear.x = 0.2;
      // }
      // else
      // {
      //   cmd_vel_msg.linear.x = 0.1;
      // ///
      // }
      publisher_cmd_vel_->publish(cmd_vel_msg);
    else
    {
      RCLCPP_INFO(this->get_logger(), "No person detected: 0");
    }
  }




  void count_callback(const darknet_ros_msgs::msg::ObjectCount::SharedPtr msg) const
  {
    if (msg->count > 3) // 3개 이상의 객체가 감지되었을 경우
    {
      RCLCPP_WARN(this->get_logger(), "Detected more than 3 objects!");
    }
  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
 {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // 필터링을 위한 Passthrough 필터 생성
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");       // Z 축으로 필터링
  pass.setFilterLimits(0.4, 1.0);     // 원하는 거리 값의 범위
  pass.filter(*cloud);

  for (const auto& point : cloud->points)
  {
    // Access Z coordinate
    float z = point.z;

    // 추가된 부분: Z 값이 0.7 이하일 때 'stop' 토픽 발행
    if (z <= 0.7)
    {
      auto message = std_msgs::msg::String();
      message.data = "stop";
      publisher_stop_->publish(message);  // 'stop' 토픽 발행
    }

    // Process Z value as needed
    RCLCPP_INFO(this->get_logger(), "Distance: %f", z);
  }
 }


  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_boxes_;
  rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscription_count_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcl_;
  // 추가된 부분: 'person' 및 'stop' 토픽 발행을 위한 Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_person_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_stop_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
