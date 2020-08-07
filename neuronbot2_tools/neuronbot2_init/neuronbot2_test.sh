#!/bin/bash

# source melodic env
if [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
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
	roslaunch rplidar_ros view_rplidar.launch
fi

if [ "$1" = "3" ]
then
    print_test_banner 3 "LED"
    roslaunch neuronbot2_led led_control.launch led_color:=9
fi

if [ "$1" = "4" ]
then
    print_test_banner 4 "GPIO"
	sudo /opt/sema/binary/linux64/bin/SEMA_GUI.sh
fi

if [ "$1" = "demo_move" ]
then
    print_test_banner 1 "Demo-Move"
    trap "exit" INT TERM ERR
    trap "kill 0" EXIT
    #roscore &
    #sleep 5
    #gnome-terminal -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=0.3"
    $PWD/scripts/spin_test.sh > /dev/null &
    roslaunch neuronbot2_bringup base_driver.launch
    wait
fi

