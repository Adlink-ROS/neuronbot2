#ifndef BASE_DRIVER_HPP_
#define BASE_DRIVER_HPP_

#if 1 
#include "rclcpp/rclcpp.hpp"
#else
// #include <ros/ros.h>
#endif

// #include <boost/shared_ptr.hpp>

// #include "neuronbot2_bringup/transport.h"
#define ROS2
#ifdef ROS2

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

#else

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <neuronbot2_msgs/RawImu.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#endif

class BaseDriver : public rclcpp::Node
{
public:

  BaseDriver();
  ~BaseDriver();
  void work_loop();

private:
  void run();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  void cmd_vel_callback();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub; 
  void publish(const rclcpp::Time & now);

  std::shared_ptr<serial::Serial> serial_;

  rclcpp::Node::SharedPtr node_handle_;
#if 1
  std::shared_ptr<Simple_dataframe> frame;
  // BaseDriverConfig bdc;
  // Simple_dataframe frame;
#else
  // std::shared_ptr<Transport> trans;
  // boost::shared_ptr<Transport> trans;
  // boost::shared_ptr<Dataframe> frame;
#endif

#if 0
  static BaseDriver* Instance()
  {
    if (instance == NULL)
      instance = new BaseDriver();

    return instance;
  }
#endif

private:
  
  std::string port;
  int32_t baudrate;

  std::string base_frame;
  std::string odom_frame;

  bool publish_tf;

  std::string cmd_vel_topic;
  std::string odom_topic;
  double odometry_frequency;

  rclcpp::TimerBase::SharedPtr tmr_odometry;
  void update_odom();
#if 1
  // auto cmd_vel_callback(const geometry_msgs::msg::Twist vel_cmd);
#else
  void cmd_vel_callback(const geometry_msgs::msg::Twist& vel_cmd);
#endif

#if 0
  //void correct_pos_callback(const std_msgs::Float32MultiArray& pos);
  void init_cmd_odom();
  void init_pid_debug();
#endif

#if 0
  void init_imu();
#endif

  void read_param();
#if 0

  void update_param();
  //void update_encoder();
  void update_speed();
  void update_pid_debug();
  void update_imu();
#endif

public:

#if 0
  BaseDriverConfig& getBaseDriverConfig(){
    return bdg;
  }

  ros::NodeHandle* getNodeHandle(){
    return &nh;
  }

  ros::NodeHandle* getPrivateNodeHandle(){
    return &pn;
  }
#endif

private:
  // BaseDriverConfig bdg;
#if 0
  static BaseDriver* instance;

  boost::shared_ptr<Transport> trans;
  boost::shared_ptr<Dataframe> frame;
#endif
#if 1
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
#else
  ros::Subscriber cmd_vel_sub;
#endif

#if 1
  // rclcpp::Publisher<nav_msgs::msg::Odometry> odom_pub;
#else
  ros::Publisher odom_pub;
#endif

#if 1
  // nav_msgs::msg::Odometry odom;
  // geometry_msgs::msg::TransformStamped odom_trans;
  // tf2::TransformBroadcaster odom_broadcaster;
#else
  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster odom_broadcaster;
#endif

#if 1
  // rclcpp::Node::SharedPtr nh;
  // rclcpp::Node::SharedPtr pn;
#else
  ros::NodeHandle nh;
  ros::NodeHandle pn;
#endif
  

// for whatever pid_debug is
#if 0
#define MAX_MOTOR_COUNT 4
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_debug_pub_input[MAX_MOTOR_COUNT];
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_debug_pub_outptu[MAX_MOTOR_COUNT];
  ros::Publisher pid_debug_pub_input[MAX_MOTOR_COUNT];
  ros::Publisher pid_debug_pub_output[MAX_MOTOR_COUNT];
  std_msgs::Int32 pid_debug_msg_input[MAX_MOTOR_COUNT];
  std_msgs::Int32 pid_debug_msg_output[MAX_MOTOR_COUNT];
#endif


  bool need_update_speed;

#if 0
  neuronbot2_msgs::RawImu raw_imu_msgs;
  auto raw_imu_msgs;
#endif

// imu publichsher
#if 1
#else
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub;
  ros::Publisher raw_imu_pub;
#endif
};
#endif // BASE_DRIVER_HPP_