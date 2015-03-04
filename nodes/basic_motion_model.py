#!/usr/bin/env python

# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3

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

### the reference provider will get the x,y from the JointState.position and get an input service from user for a desired
### x,y position and turn this into a velocity. The velocity is then sent to the controller send_vel_command, which in turn
### sends the velocity to the robot. 






				### SENDING VELOCITY COMMAND ####
### Publish the calculated linear and angular velocity to the Turtlebot in order to 
### get to the desired position that was given by the service REFERENCE PROVIDER
def send_vel_command():

	#Initiate motion model node for the Bayes Filter.
	rospy.init_node('motion_model', anonymous=True)

	#Define subscriber to get the turtlebot's state belief (bel_state)
	rospy.Subscriber("joint_states", JointState)

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
	try: send_vel_command()
	except rospy.ROSInterruptException: pass
