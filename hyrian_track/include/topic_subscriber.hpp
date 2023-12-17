#ifndef TOPIC_SUBSCRIBER_HPP_
#define TOPIC_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber();

private:
  float tracking_gain;
  float error_x;
  mutable bool is_person_detected_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  void boxes_callback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg);
  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void control_robot(float error_x);
  void stop_robot();

  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_boxes_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pcl_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_person_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_stop_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
};

#endif  // TOPIC_SUBSCRIBER_HPP_
