#!/usr/bin/env python
# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM

import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
from my_tutorial.srv import * #import all custom package srv
from my_tutorial.msg import * #import all custom package msg
from sensor_msgs.msg import LaserScan

x_est = 0
y_est = 0
th_est = 0

#create instance of a publisher
pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size = 10)

def get_state_estimate(state_est):
	print "in state estimate"
	
	global x_est, y_est, th_est

	x_est = state_est.x
	y_est = state_est.y
	th_est = state_est.th

	print "in current_estimate", x_est, y_est, th_est

	t =rospy.get_time()
	reference_request_client(t)

def reference_request_client(req):
	rospy.wait_for_service('reference_request')
	try:
		ref_state = rospy.ServiceProxy('reference_request', RefState)
		x_desired = ref_state(req).q.x
		y_desired = ref_state(req).q.y
		th_desired = ref_state(req).q.th

		desired_state = Config(x_desired, y_desired, th_desired)

		print "response from request at is: [%s %s %s]"%(x_desired, y_desired, th_desired)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e

	send_vel_command(desired_state)

def send_vel_command(desired_state):

	# set the velocities:
	# x_dot = (desired_state.x - x_est)/dt
	# y_dot = (desired_state.y - y_est)/dt
	# th_dot = (desired_state.th - th_est)/dt
	print "in send_vel_command"	
	r = rospy.Rate(10)

	linear_vel = 0.1
	angular_vel = 0.0

	velocities = Twist(Vector3(linear_vel, 0, 0),Vector3(0,0,angular_vel))	
	rospy.loginfo(velocities)
	pub.publish(velocities)

	r.sleep()

def main():
	print "in Main"
	rospy.init_node('motion_model', anonymous=True)#, log_level=rospy.INFO)
	print "node initiated"
	#rospy.Timer(rospy.Duration(0.1127), get_state_estimate, oneshot=False)
	rospy.Subscriber('state_estimate', Config, get_state_estimate)
	print "state has been gotten"
	#send_vel_command(1)
	rospy.spin()


if __name__=='__main__':
		
	try:
		main()
	except rospy.ROSInterruptException: pass