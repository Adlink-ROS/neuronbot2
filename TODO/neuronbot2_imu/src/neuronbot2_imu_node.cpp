#include "neuronbot2_imu/neuronbot2_imu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "neuronbot2_imu");
  ros::NodeHandle nh, pnh("~");
  Neuronbot2IMU neuronbot2_imu(nh, pnh);

  ros::spin();

  return 0;
}
