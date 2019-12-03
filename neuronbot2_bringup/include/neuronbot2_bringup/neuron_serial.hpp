// #pragme once
#include <chrono>
#include <utility>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
// TODO: manage header files
#include "neuronbot2_bringup/dataframe.h"
#include "neuronbot2_bringup/simple_dataframe.h"
#include "neuronbot2_bringup/data_holder.h"
#include "neuronbot2_bringup/simple_dataframe_master.h"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "serial/serial.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
// TODO: namespace

using namespace std::chrono_literals;
using duration = std::chrono::nanoseconds;

class NeuronSerial : public rclcpp::Node
{
public:
    NeuronSerial();

private:
    rclcpp::Node::SharedPtr node_handle_;
    // Send messages at this frequency to keep the node alive.
    duration keepalive_period = 100ms;
    // The frequency to poll for a uart message.
    duration uart_poll_period = 500ms;
    duration pub_period = 100ms;


    // rclcpp::Publisher<> odom raw data;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    // rclcpp::Subscription<> sub raw_commands;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr raw_mag_pub;

    // Init
    void parameter_init();
    void init_cmd_odom();
 
    void read_cb();
    void keepalive_cb();
    void on_motor_move(geometry_msgs::msg::Twist::SharedPtr msg);
    void update_odom();
    void update_imu();

    rclcpp::TimerBase::SharedPtr keepalive_timer;
    rclcpp::TimerBase::SharedPtr read_timer;
    rclcpp::TimerBase::SharedPtr pub_timer;

    std::shared_ptr<serial::Serial> serial_;
    std::shared_ptr<Simple_dataframe> frame;
    std::shared_ptr<Robot_parameter> rp;
    Data_holder *dh;
};