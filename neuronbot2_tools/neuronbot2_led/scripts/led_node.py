#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Byte
from std_msgs.msg import UInt16

def led_loop(led_color):
	rospy.init_node('led_node')
	pub1 = rospy.Publisher('table_command', Byte, queue_size = 10)
	r = rospy.Rate(30)

	if len(led_color) > 0:
		command = int(led_color)
	else:
		command = 0		

	while not rospy.is_shutdown():
		pub1.publish(command)
		r.sleep()

#------------------------------Main-------------------------------#
if __name__ == '__main__':		
	# LED color
	# '0': Black
	# '1': Amber
	# '2': Blue
	# '3': Green
	# '4': Red
	# '5': Meteor
	# '6': Rainbow
	# '7': Rainbow breathing
	# '8': Breathing
	# '9': Back and Forward	
	# print >> sys.stderr, "led_color=%s" % led_color	
	led_color = rospy.get_param('/led_control_node/led_color', '7')
	led_loop(led_color)
