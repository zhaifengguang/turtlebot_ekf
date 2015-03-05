#!/usr/bin/env python

# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3
from my_tutorial.srv import * 

			### Getting the robot's state estimate ####
### For now this is from the joint_state_publisher
### SHOULD CHANGE TO RECIEVE STATE ESTIMATE FROM MEASUREMENT_MODEL NODE
# def turtlebot_get_state_belief():

# 	#Define subscriber to get the turtlebot's state belief (bel_state)
# 	rospy.Subscriber("joint_states", JointState)

# 	#Initiate motion model node for the Bayes Filter.
# 	rospy.init_node('motion_model', anonymous=True)

	
# 	rospy.spin()


				### GETTING DESIRED POSITION INTO VELOCITY ####
### writing a velocity to robot ----> THIS FUNCTION SHOULD BE INTIRELY REPLACED BY REFERENCE PROVIDER 
### reference provider should be a SERVICE
def get_desired_state_server():

    rospy.init_node('get_desired_state_server')
    ### gets the service, and then calls the function get_vel with the servie parameters
    s = rospy.Service('get_desired_state', DesiredState, get_vel)
    print "Ready to recieve desired state."

    rospy.spin()


### the reference provider will get the x,y from the JointState.position and get an input service from user for a desired
### x,y position and turn this into a velocity. The velocity is then sent to the controller send_vel_command, which in turn
### sends the velocity to the robot. 

def get_vel(req):
	# print "You want the Turtlebot to go to coordinates x= %s, y= %s, th=%s"%(req.x, req.y, req.th)
	# return DesiredStateResponse(req.v, req.w)
	print "Returning %s+ %s = %s"%(req.x, req.y, req.th)
	### changing x and y and theta to velocity should happen here
	### return the requested type that is angular and lienar velocity

	#Initiate motion model node for the Bayes Filter.
	#rospy.init_node('motion_model', anonymous=True)

	#Define subscriber to get the turtlebot's state belief (bel_state)
	rospy.Subscriber("joint_states", JointState)

	a = JointState.position
	print a

	DesiredState.v = req.x + req.y
	DesiredState.w = req.th
	return (DesiredState.v, DesiredState.w)




				### SENDING VELOCITY COMMAND ####
### Publish the calculated linear and angular velocity to the Turtlebot in order to 
### get to the desired position that was given by the service REFERENCE PROVIDER
def send_vel_command():

	#Define publiser to send the calculated velocity to the turtlebot
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size = 10)
	
	# #Initiate motion model node for the Bayes Filter.
	# rospy.init_node('motion_model', anonymous=True)

	r = rospy.Rate(62.5) #62.5hz
	

	while not rospy.is_shutdown():


		t = rospy.get_time()
		velocity_linear=0
		velocity_angular= 0.1
		velocities = Twist(Vector3((velocity_linear),0,0), Vector3(0,0,(velocity_angular)))
		rospy.loginfo(velocities)
		pub.publish(velocities)


		r.sleep()




#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	#try: turtlebot_get_state_belief()
	try: get_desired_state_server()
	except rospy.ROSInterruptException: pass
