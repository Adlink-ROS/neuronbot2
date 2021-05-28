#!/bin/bash

# source noetic env
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# source NB2 env
if [ -f "$PWD/../../../../devel/setup.bash" ]; then
    source $PWD/../../../../devel/setup.bash
fi

function print_test_banner()
{
	echo "*****************************************"
	echo "            Test $1: $2"
	echo "*****************************************"
}

if [ "$1" = "0" ]
then
    print_test_banner 1 "Test All"
    trap "exit" INT TERM ERR
    trap "kill 0" EXIT    
    echo "Initializing..."
    sleep 1
    roscore 2> /dev/null &
    sleep 1

    # wait until roscore is runing
    check_result=$(rostopic list | grep /rosout)
    while [ -z "$check_result" ]; do
        sleep 1
        echo "wait for roscore"
        check_result=$(rostopic list | grep /rosout)
    done

    #gnome-terminal -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=0.3"
    $PWD/scripts/spin_test.sh > /dev/null &
    rviz -d $PWD/../../neuronbot2_nav/rviz/view_lidar.rviz > /dev/null &

    echo "launch bringup!"
    roslaunch neuronbot2_bringup bringup.launch
    wait
fi


if [ "$1" = "1" ]
then
	print_test_banner 1 "Base Driver"
	trap "exit" INT TERM ERR
	trap "kill 0" EXIT
	#roscore &
	#sleep 5
	#gnome-terminal -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=0.3"
    $PWD/scripts/spin_test.sh > /dev/null &
    roslaunch neuronbot2_bringup base_driver.launch
	wait
fi

if [ "$1" = "2" ]
then
	print_test_banner 2 "RPLidar"
    trap "exit" INT TERM ERR
    trap "kill 0" EXIT
    roscore > /dev/null &
    echo "Initializing..."
    sleep 3
    rviz -d $PWD/../../neuronbot2_nav/rviz/view_lidar.rviz > /dev/null &
	roslaunch neuronbot2_bringup rplidar.launch
    wait
fi

if [ "$1" = "3" ]
then
    print_test_banner 3 "LED"
    $PWD/../neuronbot2_led/scripts/led_control.py -p /dev/neuronbotLED -m 8
fi

if [ "$1" = "4" ]
then
    print_test_banner 4 "GPIO"
    sudo /opt/sema/binary/linux64/bin/SEMA_GUI.sh
fi

if [ "$1" = "demo_move" ]
then
    print_test_banner 1 "Demo Move"
    trap "exit" INT TERM ERR
    trap "kill 0" EXIT
    $PWD/scripts/move_example.sh > /dev/null &
    roslaunch neuronbot2_bringup base_driver.launch
    wait
fi


