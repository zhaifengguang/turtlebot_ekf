#!/usr/bin/env python

import sys
import rospy
from my_tutorial.srv import *

def sub_two_ints_client(x):
    rospy.wait_for_service('sub_two_ints')
    print "here"
    try:
        print "hi there"
        sub_two_ints = rospy.ServiceProxy('sub_two_ints', Sub)
        print "worked?"
        one = sub_two_ints(x).a
        two = sub_two_ints(x).b
        print "response from request x=%s and y=%s: sum=%s"%(x, one, two)
        return (one, two)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print "first"
    if len(sys.argv) == 2:
        summed = int(sys.argv[1])
        print "a"
    else:
        print usage()
        print "b"
        sys.exit(1)
    print "Requesting %s"%(summed)
    print "%s %s"%(summed,sub_two_ints_client(summed))