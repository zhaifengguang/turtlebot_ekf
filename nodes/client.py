#!/usr/bin/env python

import sys
import rospy
from my_tutorial.srv import *

def add_two_ints_client(x,y):
    rospy.wait_for_service('add_two_ints')
    print "here"
    try:
        print "hi there"
        add_two_ints = rospy.ServiceProxy('add_two_ints', Add)
        resp1 = add_two_ints(x,y)
        print "response from request x=%s and y=%s: sum=%s"%(x, y, resp1)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print "first"
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        print "a"
    else:
        print usage()
        print "b"
        sys.exit(1)
    print "Requesting %s"%(x)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x,y))