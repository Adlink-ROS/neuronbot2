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

#include <chrono>
#include <utility>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "neuronbot2_bringup/dataframe.hpp"
#include "neuronbot2_bringup/simple_dataframe.hpp"
#include "neuronbot2_bringup/data_holder.hpp"
#include "neuronbot2_bringup/simple_dataframe_master.hpp"
#include <serial/serial.h>

// TODO: namespace

using namespace std::chrono_literals;
using duration = std::chrono::nanoseconds;

namespace neuronbot2
{
class NeuronSerial : public rclcpp::Node
{
public:
    NeuronSerial();

private:
    rclcpp::Node::SharedPtr node_handle_;
    double odom_freq;
    double imu_freq;
    double cmd_vel_timeout;

    std::string odom_frame_parent;
    std::string odom_frame_child;
    std::string imu_frame;

    // Switch
    bool publish_tf_;
    bool calibrate_imu_;
    bool cmd_vel_timeout_switch;

    // imu calibration bias
    float acc_x_bias;
    float acc_y_bias;
    float acc_z_bias;
    float vel_theta_bias;        

    void parameter_init();
    void read_firmware_info();
    void keepalive_cb();
    void on_cmd_vel(geometry_msgs::msg::Twist::SharedPtr msg);
    void on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg);
    void update_odom();
    void update_imu();

    // Timer
    rclcpp::TimerBase::SharedPtr keepalive_timer;
    rclcpp::TimerBase::SharedPtr odom_read_timer;
    rclcpp::TimerBase::SharedPtr imu_read_timer;
    
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<serial::Serial> serial_;
    std::shared_ptr<Simple_dataframe> frame;
    std::shared_ptr<Robot_parameter> rp;
    std::shared_ptr<geometry_msgs::msg::Twist> motor_cmd;
    Data_holder *dh;
};
} // neuronbot2