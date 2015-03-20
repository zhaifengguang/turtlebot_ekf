#!/usr/bin/env python

import rospy
import sys
from my_tutorial.srv import * 
from my_tutorial.msg import *
from math import pi
from nav_msgs.msg import Odometry

x_now = 0
y_now = 0
th_now = 0


def reference_request_server(req):

	x_desired = 0
	y_desired = y_now + 0.00001
	#th_desired = pi/2
	th_desired = th_now

	desired_state = Config(x_desired, y_desired, th_desired)
	#rospy.loginfo(desired_state)

	return desired_state

def current_estimate(data):
	global x_now, y_now, th_now

	x_now = data.x
	y_now = data.y
	th_now = data.th
	print "Odom prediction", x_now, y_now, th_now

def main():

	#initiate service node
	rospy.init_node('reference_provider')

	rospy.Subscriber('state_estimate', Config, current_estimate)
	#create instance of server
	s = rospy.Service('reference_request', RefState, reference_request_server)
	print "Ready to provide reference configuration for the Turtlebot."



	rospy.spin()


if __name__=='__main__':
	main()