#!/usr/bin/env python

import rospy
from my_tutorial.srv import * 

def handle_get_desired_state(req):
	print "You want the Turtlebot to go to coordinates x= %s, y= %s"%(req.x, req.y)
	return DesiredStateResponse(req.x, req.y)

def get_desired_state():

    rospy.init_node('get_desired_state')
    s = rospy.Service('desired_turtlebot_pos', DesiredState,handle_get_desired_state)
    print "Ready to recieve desired state."

    rospy.spin()


if __name__ == '__main__':
	get_desired_state()


# if __name__ == '__main__':
# 	try: get_desired_state()
# 	except ROSInterruptException: pass