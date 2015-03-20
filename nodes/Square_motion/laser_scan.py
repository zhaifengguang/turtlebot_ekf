#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from my_tutorial.msg import * 

pub = rospy.Publisher('range_data',String, queue_size = 10)

#subscribed to the /scan topic. pointcloud_to_laserscan package converts kinect 3D pcl to 2D laser scan
def get_laser_range(): 
	#make node
 	rospy.init_node('laser_data', anonymous=True)
 	#make subscriber
	rospy.Subscriber("scan", LaserScan, print_laser_scan)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

def conver_to_string(data):
	string = ''
	for i in range(0,len(data)):
		string += str(data[i])+', ' 
	string += ' || len = ' + str(len(data))
	return(string)

def print_laser_scan(data):
	global pub
	msg = conver_to_string(data.ranges)
		#print "detected ranges: %s"%(data.ranges)
	msg_array = data.ranges
	rospy.sleep(1)

	pub.publish(msg_array)


if __name__ == '__main__':
	#try: turtlebot_get_state_belief()
	try: get_laser_range()
	except rospy.ROSInterruptException: pass
