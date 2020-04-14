#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <people_msgs/People.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static geometry_msgs::Twist Gcmd_vel;

void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel)
{
  ROS_INFO("I heard: [%0.3f, %0.3f, %0.3f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  Gcmd_vel = cmd_vel;
}

int main(int argc, char **argv)
{
  static bool initial = true;
  ros::init(argc, argv, "info_publisher");
  ros::NodeHandle n("~");
  std::string robot_frame  = std::string("base_link");
  std::string global_frame = std::string("world");
  n.getParam("robot_frame", robot_frame);

  ROS_INFO("robot_frame: %s", robot_frame.c_str());
  
  std::string info_topic = std::string("/people");
  n.getParam("info_topic", info_topic);
  std::string cmd_vel_topic = std::string("/cmd_vel");
  n.getParam("cmd_vel_topic", cmd_vel_topic);
  ros::Subscriber sub = n.subscribe(cmd_vel_topic, 1, cmd_vel_cb);
  
  ros::Publisher pub = n.advertise<people_msgs::People>(info_topic, 1000);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf_listener(buffer);
  ros::Time time(0);
  ros::Duration timeout(0.2);
  geometry_msgs::TransformStamped  base_link_to_world;
  ros::Rate loop_rate(10);

  while(ros::ok()) {

    try {
      base_link_to_world = buffer.lookupTransform(global_frame, robot_frame, time, timeout);
    } 
    catch (tf2::TransformException &e) {
      // handle lookup error (bypass now)
      continue;
    }
    ros::Time now = ros::Time::now();

    people_msgs::People trend;
    people_msgs::Person robot;
    trend.header.frame_id = global_frame;
    trend.header.stamp = now;
    robot.name = "";
    robot.position.x = base_link_to_world.transform.translation.x;
    robot.position.y = base_link_to_world.transform.translation.y;
    robot.position.z = base_link_to_world.transform.translation.z;
    robot.reliability = 0.90;

    geometry_msgs::PointStamped Gcmd_velq; 
    tf2::Quaternion quat;
    tf2::fromMsg(base_link_to_world.transform.rotation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x = Gcmd_vel.linear.x;
    double y = Gcmd_vel.linear.y;
    robot.velocity.x = x*cos(yaw) - y*sin(yaw);
    robot.velocity.y = x*sin(yaw) + y*cos(yaw);
    robot.velocity.z = 0;
  
    trend.people.push_back(robot);
    pub.publish(trend);

    ros::spinOnce();
		loop_rate.sleep();
  }
  return 0;
}