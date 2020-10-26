#ifndef BASE_DRIVER_CONFIG_
#define BASE_DRIVER_CONFIG_

#include <ros/ros.h>

//#define WRITE_ROBOT_PARAM
#define USE_DYNAMIC_RECONFIG
#ifdef USE_DYNAMIC_RECONFIG
#include <dynamic_reconfigure/server.h>
#include "neuronbot2_bringup/neuronbot2_driverConfig.h"
#endif

class Robot_parameter;
class BaseDriverConfig
{
public:
  BaseDriverConfig(ros::NodeHandle &p);
  ~BaseDriverConfig();

  void init(Robot_parameter* r);
  void SetRobotParameters();
#ifdef USE_DYNAMIC_RECONFIG

  void dynamic_callback(neuronbot2_bringup::neuronbot2_driverConfig &config, uint32_t level);

  bool get_param_update_flag();

private:
  dynamic_reconfigure::Server<neuronbot2_bringup::neuronbot2_driverConfig > server;
  dynamic_reconfigure::Server<neuronbot2_bringup::neuronbot2_driverConfig >::CallbackType f;
#endif
public:
  Robot_parameter* rp;
  
  std::string port;
  int32_t baudrate;

  std::string base_frame;
  std::string odom_frame;

  bool publish_tf;
  double cmd_vel_idle_time;

  std::string cmd_vel_topic;
  std::string odom_topic;
  bool out_pid_debug_enable;
private:
#ifdef USE_DYNAMIC_RECONFIG
  bool param_update_flag;
#endif
  ros::NodeHandle& pn;
  ros::ServiceClient client;

   bool set_flag;
};

#endif
