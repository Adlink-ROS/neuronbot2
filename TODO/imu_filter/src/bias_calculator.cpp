#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

double x = 0, y = 0, z = 0;
double count = 0.0;
geometry_msgs::Vector3 v;

void calculator(sensor_msgs::Imu  imu_msg) {
  x += imu_msg.angular_velocity.x;
  y += imu_msg.angular_velocity.y;
  z += imu_msg.angular_velocity.z;
  count++;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bias_calculator");
  ROS_INFO("Please keep your IMU still for 10 secs");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu", 1000, calculator);
  ros::Publisher pub = n.advertise< geometry_msgs::Vector3>("bias", 1);

  while(ros::ok()) {
    for(int i=0; i<10; i++)
       {ros::Duration(1).sleep();
        ROS_INFO("%ds left", 10-i); 
    ros::spinOnce(); }
    if(count == 0)
       { ROS_WARN("can't receive imu0 data. Please try again.");
        break; }
    v.x = x /count;
    v.y = y /count;
    v.z = z /count;
    pub.publish(v);
    ROS_INFO("Calculate ends."); 
    ROS_INFO("gyro's initial data offest is %f, y is %f, z is %f", v.x, v.y, v.z);
    break;
  }
}
  
