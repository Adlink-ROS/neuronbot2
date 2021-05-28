#!/bin/bash
LOG_FILE=/tmp/nb2_spin_test.log

echo "" > $LOG_FILE

echo "source ROS env" >> $LOG_FILE
# source 
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source /opt/ros/noetic/setup.bash
  echo "found opt" >> $LOG_FILE
fi

# wait until roscore is runing
check_result=$(rostopic list | grep /rosout)
while [ -z "$check_result" ]; do
    sleep 1
    check_result=$(rostopic list | grep /rosout)
done

# loop for movement control
for i in {1..4};
do
timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4" 
timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.4" 
done

# stop
timeout 1 rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

exit 0
