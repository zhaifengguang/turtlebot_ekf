#!/usr/bin/env python
# CONTROLLER NODE ___ THE MOTION MODEL OF THE LOCALISATION ALGORITHM

import sys
import rospy
from geometry_msgs.msg import Twist, Vector3
from my_tutorial.srv import * #import all custom package srv
from my_tutorial.msg import * #import all custom package msg
from sensor_msgs.msg import LaserScan


#create instance of a publisher
pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size = 10)




#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':

	try: get_state_estimate()
	except rospy.ROSInterruptException: pass