#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Byte
from std_msgs.msg import UInt16

class led_ctrl_node:
	def __init__(self):
		self.LED_BLACK = 0
		self.LED_WHITE = 1
		self.LED_AMBER = 2
		self.LED_BLUE = 3
		self.LED_GREEN = 4
		self.LED_RED = 5
		self.LED_METEOR = 6
		self.LED_RAINBOW = 7
		self.LED_RAINBOW_BREATHING = 8
		self.LED_BREATHING = 9
		self.LED_BACK_AND_FORWARD = 10
		rospy.init_node('led_node')
		self.rate = rospy.Rate(30)
		self.pub = rospy.Publisher('table_command', Byte, queue_size = 10)
		rospy.on_shutdown(self.onclosing)

	def led_loop(self, led_color):
		if len(led_color) > 0:		
			command = int(led_color)
		else:
			command = self.LED_BLACK		

		while not rospy.is_shutdown():
			self.pub.publish(command)
			self.rate.sleep()

	def onclosing(self):
		print >> sys.stderr, "shutdown time!"
		self.pub.publish(self.LED_AMBER)


#------------------------------Main-------------------------------#
if __name__ == '__main__':
	node = led_ctrl_node()
	led_color = rospy.get_param('/led_control_node/led_color', '3')	
	node.led_loop(led_color)
