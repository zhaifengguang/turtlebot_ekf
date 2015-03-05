#!/usr/bin/env python

import rospy
import sys
from my_tutorial.srv import * 

def get_desired_state_client(x,y,th):
	rospy.wait_for_service('get_desired_state')
	try: 
		get_desired_state = rospy.ServiceProxy('get_desired_state', DesiredState)                                                                                                                                     
		resp1 = get_desired_state(x, y, th)
		return (resp1.v, resp1.w)

	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e

def usage():
	return "%s [x y th]"%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 4:
		x = int(sys.argv[1])
		y = int(sys.argv[2])
		th = int(sys.argv[3])

	else: 
		print usage()
		sys.exit(1)

	print "Requesting [%s %s %s]"%(x,y,th)
	print "%s %s %s %s"%(x,y,th,get_desired_state_client(x,y,th))

