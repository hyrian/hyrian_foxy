#include "rclcpp/rclcpp.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/object_count.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> // 추가된 부분
#include "std_msgs/msg/string.hpp"  // 추가된 부분
#include "geometry_msgs/msg/twist.hpp"

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber()
    : Node("topic_subscriber"),tracking_gain(0.3),error_x(0.0), cmd_vel_msg(geometry_msgs::msg::Twist()), too_close_detected(false)
  {
    this->declare_parameter("ros_domain_id", 30);
    this->get_parameter("ros_domain_id", ros_domain_id_);

    this->subscription_boxes_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
      "darknet_ros/bounding_boxes", 10, std::bind(&TopicSubscriber::boxes_callback, this, std::placeholders::_1));
    this->subscription_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/depth/points", 10, std::bind(&TopicSubscriber::pcl_callback, this, std::placeholders::_1));
    // subscription_count_ = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
    //   "darknet_ros/found_object", 10, std::bind(&TopicSubscriber::count_callback, this, std::placeholders::_1));
    
     // 추가된 부분: 'person'이 감지되면 'person' 토픽 발행

     // 추가된 부분: Z 값이 0.7 이하로 감지되면 'stop' 토픽 발행
    this->publisher_person_ = this->create_publisher<std_msgs::msg::String>("person", 10);
    this->publisher_stop_ = this->create_publisher<std_msgs::msg::String>("stop", 10);
    this->publisher_wait_ = this->create_publisher<std_msgs::msg::String>("wait", 10);
    this->publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
    this->publisher_good_ = this->create_publisher<std_msgs::msg::String>("good", 10);
  }

private:
  float tracking_gain;
  float error_x;
  float distance = 0.0;
  float now_distance = 0; // 이 부분을 추가하세요.
  mutable bool is_person_detected_;
  bool too_close_detected;
  geometry_msgs::msg::Twist cmd_vel_msg;
  int ros_domain_id_;

  void boxes_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) //방향 제어 
  {
    float image_center_x = 320.0;
    // float error_x = 0.0;
    // // bool detected_person = false;
    // bool is_person_detected_ = false;
    this->error_x = 0.0;
    this->is_person_detected_ = false;
    
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pointcloud_msg, *cloud);


    for (const auto& boundingBox : msg->bounding_boxes)
    {
      if (boundingBox.class_id == "person")
      { 
        // RCLCPP_INFO(this->get_logger(), "Person");
        // this->detected_person = true;
        this->is_person_detected_ = true;
        float box_center_x = (boundingBox.xmax + boundingBox.xmin) / 2.0;
        this->error_x = box_center_x - image_center_x;
        RCLCPP_INFO(this->get_logger(), "error_x: %f", this->error_x);
        std::async(std::launch::async, &TopicSubscriber::human_tracking, this);
        break;
      }
    }

  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) //정지
  { 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // 필터링을 위한 Passthrough 필터 생성
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");       // Z 축으로 필터링
    pass.setFilterLimits(0.4, 1.5);     // 원하는 거리 값의 범위
    pass.filter(*cloud);
    float previous_distance = 0; 
    if (cloud->points.empty())
    {
      this->distance = 0;
    }
    else
    {
      float min_distance = std::numeric_limits<float>::max();
      
      for (const auto& point : cloud->points)
      {
        float z = point.z;
        if (z < min_distance)
        {
          min_distance = z;
        }
      }
      if (min_distance > 8.0) // SOME_LARGE_VALUE는 적절한 임계값으로 설정합니다.
      {
        this->distance = previous_distance; // 거리가 너무 클 경우, 이전 거리를 사용합니다.
      }
      else
      {
        this->distance = min_distance;
        previous_distance = min_distance; // 거리가 너무 크지 않을 경우, 이전 거리를 업데이트합니다.
      }
    }
    RCLCPP_INFO(this->get_logger(), "Minimum distance: %f", this->distance);
  }
  

  void human_tracking()
  {
      if (!this->is_person_detected_) // 사람이 감지되지 않으면 로봇을 멈춥니다.
      {
        RCLCPP_INFO(this->get_logger(), "not start");
        stop_robot();
        return;
      }
      // if(this->is_person_detected_){
      //   RCLCPP_INFO(this->get_logger(), "follow start, move");
      //   auto cmd_vel_msg = geometry_msgs::msg::Twist();
      //   float tracking_gain = 0.001;
      //   cmd_vel_msg.linear.x = 0.15;
      //   cmd_vel_msg.angular.z = -tracking_gain * this->error_x;
      //   publisher_cmd_vel_->publish(cmd_vel_msg);

      //   if(this->distance <= 1.0) // 일시정지
      //   {
      //     if(this->distance <= 0.6){ // 후진
      //       RCLCPP_INFO(this->get_logger(), "too close! robot backward");
      //       auto cmd_vel_msg = geometry_msgs::msg::Twist();
      //       float tracking_gain = 0.0015;
      //       cmd_vel_msg.linear.x = -0.15;
      //       cmd_vel_msg.angular.z = -tracking_gain * this->error_x;
      //       publisher_cmd_vel_->publish(cmd_vel_msg);
      //     }
      //     RCLCPP_INFO(this->get_logger(), "100cm, robot wait");
      //     wait_robot();
      //   }
      // }


       if (!this->too_close_detected && this->distance <= 0.7 && this->distance >= 0.58) // 처음으로 너무 가까워지면
      {
        RCLCPP_INFO(this->get_logger(), "too close for the first time! Publishing 'good' topic");
        auto good_msg = std_msgs::msg::String();
        good_msg.data = "good";
        publisher_good_->publish(good_msg);
        this->too_close_detected = true; // too_close_detected를 true로 설정하여 다시 발행하지 않도록 합니다.
      }

      if(this->distance < 0.55 && this->distance >= 0.4) // 후진
      {
        RCLCPP_INFO(this->get_logger(), "too close! robot backward");
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        float tracking_gain = 0.0015;
        cmd_vel_msg.linear.x = -0.1;
        cmd_vel_msg.angular.z = -tracking_gain * this->error_x;
        publisher_cmd_vel_->publish(cmd_vel_msg);
      }
      else if (this->distance <= 0.7 && this->distance >= 0.58)// 사람과의 거리가 0.7 이하면 로봇을 멈춥니다.
      { 
        RCLCPP_INFO(this->get_logger(), "too close");
        stop_robot();
      }
      else if (this->distance > 0.72)// 그렇지 않으면 사람을 따라 움직입니다.
      { 
        RCLCPP_INFO(this->get_logger(), "move");

        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        float tracking_gain = 0.001;
        cmd_vel_msg.linear.x = 0.15;
        cmd_vel_msg.angular.z = -tracking_gain * this->error_x;
        publisher_cmd_vel_->publish(cmd_vel_msg);
        // break;
      }
  }

  // 함수 부분
  void control_robot(float error_x)
    {
      float k_p = 0.1;
      float angular_vel = -k_p * error_x;
      auto cmd_vel_msg = geometry_msgs::msg::Twist();
      cmd_vel_msg.linear.x = 0.15;
      cmd_vel_msg.angular.z = angular_vel;
      publisher_cmd_vel_->publish(cmd_vel_msg);
    }

    void stop_robot()
    {
      auto stop_msg = std_msgs::msg::String();
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      stop_msg.data = "stop";
      publisher_cmd_vel_->publish(cmd_vel_msg);
      publisher_stop_->publish(stop_msg);
    }

    void wait_robot()
    {
      auto wait_msg = std_msgs::msg::String();
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      wait_msg.data = "wait";
      publisher_cmd_vel_->publish(cmd_vel_msg);
      publisher_wait_->publish(wait_msg);
      std::this_thread::sleep_for(std::chrono::seconds(3)); // 3초 동안 대기
      stop_robot();
    }



 // 함수 부분
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_boxes_;
  // rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscription_count_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcl_;
  // 추가된 부분: 'person' 및 'stop' 토픽 발행을 위한 Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_person_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_wait_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_good_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}


