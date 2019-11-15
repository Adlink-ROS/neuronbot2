#include <ros/ros.h>
#include "neuronbot2_bringup/base_driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "neuronbot2_driver");
    
    BaseDriver::Instance()->work_loop();

    ros::spin();

    return 0;
}
