#!/bin/bash

# source 
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  source /opt/ros/noetic/setup.bash
fi

if [ -f "$PWD/../../../devel/setup.bash" ]; then
    source $PWD/../../../devel/setup.bash
fi

roslaunch neuronbot2_slam gmapping_move.launch open_rviz:=true  > /dev/null &

sleep 5

bash $PWD/scripts/goal_test.sh

wait
