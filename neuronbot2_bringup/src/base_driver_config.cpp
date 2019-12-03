#include "neuronbot2_bringup/base_driver_config.hpp"

#include "neuronbot2_bringup/data_holder.h"
#define PI 3.1415926f


BaseDriverConfig::BaseDriverConfig(rclcpp::Node::SharedPtr p) : pn(p)
{
#ifdef USE_DYNAMIC_RECONFIG
  param_update_flag = false;
#endif

  set_flag = true;
}


BaseDriverConfig::~BaseDriverConfig()
{
    
}

void BaseDriverConfig::init(Robot_parameter* r) 
{
  rp = r;

  //comm param
#ifdef ROS2
  std::string port = pn->declare_parameter("port", "/dev/neuronbot2");
  int32_t baudrate = pn->declare_parameter("baudrate", 115200);
  RCLCPP_INFO(pn->get_logger(), "port:%s buadrate:%d", port.c_str(), baudrate);

  std::string base_frame = pn->declare_parameter("base_frame", "base_link");
  std::string odom_frame = pn->declare_parameter("odom_frame", "odom");
  bool publish_tf = pn->declare_parameter("publish_tf", true);

  std::string cmd_vel_topic = pn->declare_parameter("cmd_vel_topic", "cmd_vel");
  std::string odom_topic = pn->declare_parameter("odom_topic", "odom");
  printf("inside config init => port: '%s' \n", port.c_str());
#else
  pn.param<std::string>("port", port, "/dev/ttyACM0");
  pn.param<int32_t>("buadrate", buadrate, 115200);
  
  ROS_INFO("port:%s buadrate:%d", port.c_str(), buadrate);

  pn.param<std::string>("base_frame", base_frame, "base_link");
  pn.param<std::string>("odom_frame", odom_frame, "odom");
  pn.param<bool>("publish_tf", publish_tf, true);

  pn.param<bool>("out_pid_debug_enable", out_pid_debug_enable, false);
  ROS_INFO("out_pid_debug_enable:%d", out_pid_debug_enable);

  //topic name param
  pn.param<std::string>("cmd_vel_topic", cmd_vel_topic, "cmd_vel");
  pn.param<std::string>("odom_topic", odom_topic, "odom");
#endif
}

void BaseDriverConfig::SetRobotParameters() 
{
#ifdef USE_DYNAMIC_RECONFIG
  static bool flag=true;
  if (flag)
  {
    flag = false;
    f = boost::bind(&BaseDriverConfig::dynamic_callback, this, _1, _2);
    server.setCallback(f);
  }
#endif
}

#ifdef USE_DYNAMIC_RECONFIG
void BaseDriverConfig::dynamic_callback(neuronbot2_bringup::neuronbot2_driverConfig &config, uint32_t level) {
  if (set_flag)
  {
    set_flag = false;
    config.wheel_diameter = rp->wheel_diameter;
    config.wheel_track = rp->wheel_track;
    config.do_pid_interval = rp->do_pid_interval;
    config.encoder_resolution = rp->encoder_resolution;
    config.kp = rp->kp;
    config.ki = rp->ki;
    config.kd = rp->kd;
    config.ko = rp->ko;
    config.cmd_last_time = rp->cmd_last_time;
    config.max_v_liner_x = rp->max_v_liner_x;
    config.max_v_liner_y = rp->max_v_liner_y;
    config.max_v_angular_z = rp->max_v_angular_z;
    config.imu_type = rp->imu_type;
    
    return;
  }
  ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %d %d %d %d %d %d %d", 
          config.wheel_diameter, config.wheel_track,  config.encoder_resolution,
          config.do_pid_interval, config.kp, config.ki, config.kd, config.ko, 
          config.cmd_last_time, 
          config.max_v_liner_x, config.max_v_liner_y, config.max_v_angular_z, 
          config.imu_type);

  rp->wheel_diameter = config.wheel_diameter;
  rp->wheel_track = config.wheel_track;
  rp->do_pid_interval = config.do_pid_interval;
  rp->encoder_resolution = config.encoder_resolution;
  rp->kp = config.kp;
  rp->ki = config.ki;
  rp->kd = config.kd;
  rp->ko = config.ko;
  rp->cmd_last_time = config.cmd_last_time;
  rp->max_v_liner_x = config.max_v_liner_x;
  rp->max_v_liner_y = config.max_v_liner_y;
  rp->max_v_angular_z = config.max_v_angular_z;
  rp->imu_type = config.imu_type;

  param_update_flag = true;
}

bool BaseDriverConfig::get_param_update_flag()
{
  bool tmp=param_update_flag;
  param_update_flag = false;
  
  return tmp;
}
#endif
