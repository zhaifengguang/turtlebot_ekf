#!/usr/bin/env python

# REFERENCE PROVIDER NODE ___ PROVIDES A PATH/MAP FOR THE TURTLEBOT TO FOLLOW. 
# The code in this node could be from a predefined map obtained from any path
# planning algorithm, such as A*, or anything else as long as it returns a desired
# state(x, y, th)
# I plan on adding an A* path planning algorithm here. 
# For now, the Turtlebot is commanded to follow a straight line at a certain distance
# from along a wall. 
# 
# Subscribed to: /state_estimate 
# Service provider: responds to the reference_request service 
# 
#
# Author: Mahdieh Nejati Javaremi 
# m.nejati@u.northwestern.edu
# Winter 2015


# Import necessary libraries and populate namespaces:  
import rospy
import sys
from my_tutorial.srv import * 
from my_tutorial.msg import *
from math import pi
from nav_msgs.msg import Odometry


# Global variables: 
x_now = 0
y_now = 0
th_now = 0


# Service the client by providing the next position of the Turtlebot's path. 
# A* will go here. 
def reference_request_server(req):

	# Only move in the y direction in 1 mm increments. 
	x_desired = x_now
	y_desired = y_now + 0.1
	th_desired = th_now

	# Package desired state in a custom message of type 'Config' 
	desired_state = Config(x_desired, y_desired, th_desired)
	
	#rospy.loginfo(desired_state)

	return desired_state



# Get the current state estimate from the measurement model in order to provide
# an appropriate next state. 
def current_estimate(data):
	global x_now, y_now, th_now

	x_now = data.x
	y_now = data.y
	th_now = data.th
	


# Initialize service. 
def main():

	# Initiate node.
	rospy.init_node('reference_provider')

	# Create instance of 
	rospy.Subscriber('state_estimate', Config, current_estimate)
	
	# Create instance of server. 
	s = rospy.Service('reference_request', RefState, reference_request_server)
	print "Ready to provide reference configuration for the Turtlebot."


	rospy.spin()


if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass