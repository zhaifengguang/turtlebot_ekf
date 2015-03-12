#!/usr/bin/env python
# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM

import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
from my_tutorial.srv import * #import all custom package srv
from my_tutorial.msg import * #import all custom package msg
from math import sqrt, pi
from sensor_msgs.msg import LaserScan

# global variable FIND A BETTER WAY TO DO THIS!
# x_estimated = 0
# y_estimated = 0
# yaw_estimated = 0
#Define publiser to send the calculated velocity to the turtlebot

pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size = 10)
clearance = 0
#Subscribed to Topic state_estimate that is being published
#by the prediction node (executable file name: data_processing)
def get_state_belief():

	#Initiate motion model node for the Bayes Filter.
	rospy.init_node('motion_model', anonymous=True)

	rospy.Subscriber('range_data', LaserScan, avoid_obstacle)
	print "back again"
	#(x_estimated, y_estimated, yaw_estimated) = get_state_belief()
	rospy.Subscriber('state_estimate', Config, send_vel_command)

	rospy.spin()


def get_desired_state_client(bel_state):

	rospy.wait_for_service('get_desired_state')
	try: 

		new_config = rospy.ServiceProxy('get_desired_state', DesiredState)
		x_desired = new_config(bel_state).q.x
		y_desired = new_config(bel_state).q.y
		yaw_desired = new_config(bel_state).q.th
		print "response from request at is: [%s %s %s]"%(x_desired, y_desired, yaw_desired)
		return (x_desired, y_desired, yaw_desired)
		#return (new_config)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e

def avoid_obstacle(data):
#def avoid_obstacle():
	
	global clearance
	#if (min(data)< 0.01):
	clearance = 0

	#else: clearance = 1
	#return(clearance)
	

def send_vel_command(data):

	global pub
	#global clearance

	r = rospy.Rate(80) #62.5hz

	x_estimated = data.x
	y_estimated = data.y
	yaw_estimated = data.th
	print "x_estimated:%s, y_estimated:%s, yaw_estimated: %s"%(x_estimated, y_estimated, yaw_estimated)
	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()
	
	t = 0

	#################

	##################

	# #clearance = avoid_obstacle()

	# if (clearance == 0): 
	# 	velocity_linear = 0
	# 	velocity_angular = 0
	# else: 
	T = 10
	r = 0.5
	velocity_linear = 2*pi*r/T
	velocity_angular = 2*pi/T
	while not rospy.is_shutdown():	

		current_pose = Config(x_estimated, y_estimated, yaw_estimated)
		print "current pose: %s"%(current_pose)
	# 	(x_desired, y_desired, yaw_desired) = get_desired_state_client(current_pose)
	# 	# print "6"
	# 	new_time = rospy.get_time()
	# 	dt = new_time - t
	# 	v_x = (x_desired - x_estimated)/dt
	# 	v_y = (y_desired - y_estimated)/dt
	# 	velocity_linear = sqrt(v_x**2 + v_y**2)
	# 	# velocity_angular= (yaw_desired - yaw_estimated)/dt


	# 	# r.sleep()

	# 	print "velocities linear=%d, angular=%d"%(velocity_linear, velocity_angular)
		velocities = Twist(Vector3((velocity_linear),0,0), Vector3(0,0,(velocity_angular)))
		rospy.loginfo(velocities)
		pub.publish(velocities)
	# 	#past_time = new_time

	# 	#r.sleep()
	# 	t = rospy.get_time()

	# rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':

	try: get_state_belief()
	except rospy.ROSInterruptException: pass


