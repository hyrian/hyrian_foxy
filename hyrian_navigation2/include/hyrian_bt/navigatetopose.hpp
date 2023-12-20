#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/string.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "behaviortree_cpp_v3/action_node.h"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <angles/angles.h>
#include <cmath>

class NavigateToPose : public BT::AsyncActionNode, public rclcpp::Node
{
public:
    NavigateToPose(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), Node("navigate_to_pose_action_node")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_arrive_pub_ = this->create_publisher<std_msgs::msg::String>("goal_arrive", 10); // Publisher for goal arrival
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&NavigateToPose::odomCallback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("Goal") };
    }

    BT::NodeStatus tick() override
    {
        std::string goal;
        if (!getInput<std::string>("Goal", goal))
        {
            throw BT::RuntimeError("NavigateToPose requires a goal");
        }

        std::vector<std::string> parts;
        std::stringstream ss(goal);
        std::string part;
        while (std::getline(ss, part, ';'))
        {
            parts.push_back(part);
        }

        if (parts.size() != 3)
        {
            throw BT::RuntimeError("The goal should be in the format 'x;y;yaw'");
        }

        double target_x = std::stod(parts[0]);
        double target_y = std::stod(parts[1]);  
        double target_yaw = std::stod(parts[2]);

        // Step 1: Rotate to face the target direction
        if (!isFacingTarget(target_x, target_y, 0.174))
        {
            rotateTowards(target_x, target_y);
            // RCLCPP_INFO(nh->get_logger(), "rotateTowards");
            RCLCPP_INFO(this->get_logger(), "rotateTowards");


            return BT::NodeStatus::RUNNING;
        }

        // Step 2: Move forward to reach the target position
        if (!isAtTargetPosition(target_x, target_y, 0.05))
        {
            moveForward();
            RCLCPP_INFO(this->get_logger(), "moveForward");

            // RCLCPP_INFO(nh->get_logger(), "moveForward");

            return BT::NodeStatus::RUNNING;
        }

        // Step 3: Rotate to reach the target yaw
        if (!isAtTargetYaw(target_yaw, 10))
        {
            rotateToYaw(target_yaw);
            RCLCPP_INFO(this->get_logger(), "rotateToYaw");

            return BT::NodeStatus::RUNNING;
        }
        
         if (isAtTargetPosition(target_x, target_y, 0.05) && isAtTargetYaw(target_yaw,10))
        {
            std_msgs::msg::String message;
            message.data = "Goal reached";
            goal_arrive_pub_->publish(message); // Publishing the goal arrival message
            RCLCPP_INFO(this->get_logger(), "Goal reached, message published on 'goal_arrive' topic");

            // Stop the robot once the target pose is reached
            sendVelocityCommand(0.0, 0.0);
            return BT::NodeStatus::SUCCESS;
        }
        // Stop the robot once the target pose is reached

        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_arrive_pub_;
    double current_x_;
    double current_y_;
    double current_yaw_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void sendVelocityCommand(double linear, double angular)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear;
        cmd_vel_msg.angular.z = angular;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    bool isFacingTarget(double target_x, double target_y, double tolerance)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        double target_angle = std::atan2(dy, dx);
        double angle_diff = angles::normalize_angle(target_angle - current_yaw_);
        RCLCPP_INFO(this->get_logger(), "Angle difference: %f", angle_diff);
        return std::abs(angle_diff) < tolerance;
    }

    void rotateTowards(double target_x, double target_y)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        double target_angle = std::atan2(dy, dx);
        double angle_diff = angles::normalize_angle(target_angle - current_yaw_);
        double angular_velocity = (angle_diff > 0) ? 0.4 : -0.4;
        sendVelocityCommand(0.0, angular_velocity);
    }

    bool isAtTargetPosition(double target_x, double target_y, double tolerance)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance < tolerance;
    }

    void moveForward()
    {
        sendVelocityCommand(0.1, 0.0); // Move forward with a fixed linear velocity
    }

    bool isAtTargetYaw(double target_yaw, double tolerance)
    {
        double angle_diff = angles::normalize_angle(target_yaw - current_yaw_);
        return std::abs(angle_diff) < tolerance;
    }

    void rotateToYaw(double target_yaw)
    {
        double angle_diff = angles::normalize_angle(target_yaw - current_yaw_);
        double angular_velocity = (angle_diff > 0) ? 0.4 : -0.4;
        sendVelocityCommand(0.0, angular_velocity);
    }
};