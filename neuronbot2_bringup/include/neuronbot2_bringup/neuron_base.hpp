#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;
using duration = std::chrono::nanoseconds;

class NeuronBase : public rclcpp::Node
{
public:
    NeuronBase();
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::TimerBase::SharedPtr update_odom_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    // rclcpp::Publisher<tf2_ros::TransformBroadcaster>::UniquePtr tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // temporary data
    geometry_msgs::msg::Twist motor_cmd;
    nav_msgs::msg::Odometry odom_; 
    geometry_msgs::msg::TransformStamped odom_tf;

    void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void update_odom();

    // Switch
    bool publish_tf_;
};