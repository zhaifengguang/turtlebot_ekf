#!/usr/bin/env python

from my_tutorial.srv import *
import rospy
import sys

def handle_sub_two_ints(req):
	print "here"
	resp1 = req.sum - 1
	resp2 = req.sum -2
	return (resp2, resp1)

    

def sub_two_ints_server():
    rospy.init_node('sub_two_ints_server')
    s = rospy.Service('sub_two_ints', Sub, handle_sub_two_ints)
    print "Ready to sub two ints."
    rospy.spin()

if __name__ == "__main__":
    sub_two_ints_server()

