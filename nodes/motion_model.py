#!/usr/bin/env python

# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM
# 
# Subscribed to: /state_estimate 
# Service client: requests next state over reference_request server
# Publishes to: /cmd_vel_mux/input/teleop
#
#
# Author: Mahdieh Nejati Javaremi 
# m.nejati@u.northwestern.edu
# Winter 2015


# Importing relevent libraries and populating namespaces: 
import sys
import rospy
from math import sqrt
from geometry_msgs.msg import Twist, Vector3
from my_tutorial.srv import * #import all custom package srv
from my_tutorial.msg import * #import all custom package msg
from sensor_msgs.msg import LaserScan

# Global variables: 
# I will make them local eventually ...
x_est = 0
y_est = 0
th_est = 0
old_time = 0

#Create instance of a publisher, sending a message of type Twist to the topic: /cmd_vel_mux/input/teleop
pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size = 10)

# Get the updated state(x, y, theta) estimate from the measurement model.
# Call service provider to get next desired state. 
def get_state_estimate(state_est):
	
	global x_est, y_est, th_est, old_time

	x_est = state_est.x
	y_est = state_est.y
	th_est = state_est.th
	
	t =rospy.get_time()
	old_time = t

	reference_request_client(t)



# Get the desired next state from the refernce_provider service provider. 
# Once the desired state has been recieved, call the velocity command
# function with the next desired state. 
def reference_request_client(req):

	rospy.wait_for_service('reference_request')
	try:
		ref_state = rospy.ServiceProxy('reference_request', RefState)
		x_desired = ref_state(req).q.x
		y_desired = ref_state(req).q.y
		th_desired = ref_state(req).q.th

		#Package desired state as a custom message of type 'Config' 
		desired_state = Config(x_desired, y_desired, th_desired)

		#rospy.loginfo(x_desired, y_desired, th_desired)

	except rospy.ServiceException, e: 
		rospy.logerr("Service call failed. The error message is: %s", e)

	send_vel_command(desired_state)



# Using our estimate of where we are now and the next desired position, calculate
# the desired velocity to get us ther over a certian time-step.
def send_vel_command(desired_state):

	global old_time

	# get the time step
	t_now = rospy.get_time()
	#dt = t_now - old_time
	#Using 1 second for now... timesteps were too small and the velocities too large. !fix the logic!
	dt = 1

 	r = rospy.Rate(10)

	# calculating velocities: 
	x_dot = (desired_state.x - x_est)/dt
	y_dot = (desired_state.y - y_est)/dt
	th_dot = (desired_state.th - th_est)/dt
	
	# linear_vel = sqrt(x_dot**2 + y_dot**2)
	# angular_vel = th_dot
	# Setting velocities to zero while I test my covariance plots ...
	linear_vel = 0.0
	angular_vel = 0.0

	# Send velocity commands to the /mobile_base_nodelet_manager 
	velocities = Twist(Vector3(linear_vel, 0, 0),Vector3(0,0,angular_vel))	
	rospy.loginfo(velocities)
	pub.publish(velocities)

	# spin() simply keeps python from exiting until this node is stopped
	r.sleep()

# Initialise node. 
def main():
	
	rospy.init_node('motion_model', anonymous=True)#, log_level=rospy.INFO)
	
	# Create instance of subscriber to the state_estimate topic published by the measurement model: 
	# send recieved data to the callback function "get_state_estimate"
	rospy.Subscriber('state_estimate', Config, get_state_estimate)
	
	rospy.spin()

# Fist place the code goes to when run
if __name__=='__main__':
		
	try:
		main()
	except rospy.ROSInterruptException: pass