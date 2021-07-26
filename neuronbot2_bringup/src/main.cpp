#include <ros/ros.h>
#include "neuronbot2_bringup/base_driver.h"

int main(int argc, char *argv[])
{
    system("/home/ros/neuronbot2_ros1_ws/src/neuronbot2/neuronbot2_tools/neuronbot2_led/scripts/led_control.py --port /dev/neuronbotLED --mode 5");

    ros::init(argc, argv, "neuronbot2_driver");
    
    BaseDriver::Instance()->work_loop();

    ros::spin();

    system("/home/ros/neuronbot2_ros1_ws/src/neuronbot2/neuronbot2_tools/neuronbot2_led/scripts/led_control.py --port /dev/neuronbotLED --mode 2");

    return 0;
}
