#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include "base_driver_config.h"


#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include "neuronbot2_bringup/transport.h"
#include "neuronbot2_bringup/dataframe.h"
#include <neuronbot2_msgs/RawImu.h>

class BaseDriver
{
private:
  BaseDriver();

public:
  static BaseDriver* Instance()
  {
    if (instance == NULL)
      instance = new BaseDriver();

    return instance;
  }
  ~BaseDriver();
  void work_loop();
private:
  void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
  void init_cmd_odom();
  void init_pid_debug();
  void init_imu();
  void init_param();
  void read_param();

  void update_param();
  void update_odom();
  void update_speed();
  void update_pid_debug();
  void update_imu();

public:

  BaseDriverConfig& getBaseDriverConfig(){
    return bdg;
  }

  ros::NodeHandle* getNodeHandle(){
    return &nh;
  }

  ros::NodeHandle* getPrivateNodeHandle(){
    return &pn;
  }
private:
  static BaseDriver* instance;

  BaseDriverConfig bdg;
  boost::shared_ptr<Transport> trans;
  boost::shared_ptr<Dataframe> frame;

  ros::Subscriber cmd_vel_sub;
  ros::Publisher odom_pub;

  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster odom_broadcaster;
  
  ros::NodeHandle nh;
  ros::NodeHandle pn;
  
#define MAX_MOTOR_COUNT 4
  ros::Publisher pid_debug_pub_input[MAX_MOTOR_COUNT];
  ros::Publisher pid_debug_pub_output[MAX_MOTOR_COUNT];

  std_msgs::Int32 pid_debug_msg_input[MAX_MOTOR_COUNT];
  std_msgs::Int32 pid_debug_msg_output[MAX_MOTOR_COUNT];

  bool need_update_speed;
  double last_cmd_vel_time;

  //neuronbot2_msgs::RawImu raw_imu_msgs;
  sensor_msgs::Imu raw_imu_msgs;
  sensor_msgs::MagneticField raw_mag_msgs;
  ros::Publisher raw_imu_pub;
  ros::Publisher raw_mag_pub;
};
