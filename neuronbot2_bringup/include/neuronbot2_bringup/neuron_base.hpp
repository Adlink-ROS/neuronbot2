//  Copyright 2020 ADLINK Technology, Inc.
//  Developer: YU-WEN, CHEN (real.yuwen@gmail.com)
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/imu.hpp"

namespace neuronbot2
{

using namespace std::chrono_literals;
using duration = std::chrono::nanoseconds;

class NeuronBase : public rclcpp::Node
{
public:
    NeuronBase();
private:
    // Callback function
    void on_cmd_vel(geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void on_raw_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void update_status();

    // Switch
    bool publish_tf_;
    bool calibrate_imu_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_status_timer;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    // imu calibration bias
    float acc_x_bias;
    float acc_y_bias;
    float acc_z_bias;
    float vel_theta_bias;
   
    // Pending data
    geometry_msgs::msg::Twist motor_cmd;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::Imu imu_data_; 
    geometry_msgs::msg::TransformStamped odom_tf;
};
} //neuronbot2