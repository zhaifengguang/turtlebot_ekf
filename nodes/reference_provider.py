#!/usr/bin/env python

import rospy
import sys
from my_tutorial.srv import * 
from my_tutorial.msg import *
from math import cos, sin,atan

####
#	global variables: 
r = 0.5 #radius of circular path
h = 1
k = 1

def provide_reference_config(req):
	# x_desired = cos(req.t)
	# y_desired = sin(req.t)
	# if y_desired != 0:
	# 	th_desired = atan(x_desired/y_desired)
	# else: th_desired = 0
	if (req.x<1 and req.y < 1 and req.th < 1.57):
		x_desired = 1
		y_desired = 0
		th_desired = 0
	elif (req.x>=1 and req.y < 1 and req.th <1.57):
		x_desired = 1
		y_desired = 1
		th_desired = 1.57
	elif (req.x>=1 and req.y >= 1 and req.th>=1.57):
		x_desired = 0
		y_desired = 1
		th_desired = -3.14
	elif (req.x<1 and req.y >= 1 and req.th<= -3.14):
		x_desired = 0
		y_desired = 0
		th_desired = -1.57		
	print "Returning [%s %s %s]"%(x_desired,  y_desired, th_desired)
	desired_config = Config(x_desired, y_desired, th_desired)
	rospy.loginfo(desired_config)

    # return a new config
	return desired_config

def get_desired_state_server():

	rospy.init_node('get_desired_state_server')

    ### gets the service request (t), and then calls the function provide_reference_config to 
    ### send a response (congiguration q)
	s = rospy.Service('get_desired_state', DesiredState, provide_reference_config)
	print "Ready to provide reference configuration for the Turtlebot."
	rospy.spin()

if __name__ == "__main__":

	try:get_desired_state_server()
	except rospy.ROSInterruptException: pass


