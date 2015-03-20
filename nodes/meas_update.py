#!/usr/bin/env python

#populating namespace
import rospy
import sys
import numpy
import std_msgs.msg
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from my_tutorial.msg import * 
from tf.transformations import euler_from_quaternion



#In the extended kalman filter, the observation(measurement) model
#does not need to be a linear function of state, but may instead be 
#a differentiable function. 

#Simplest (and most error prone) measurmement model using scan data. 
#We are throwing most of the data away for the sake of simplicity. 
#if the PCL was used instead of laser scan, the measurement update model
#would reduce the uncertainty more. 

#This is just a simple example. I am trying to use kinect info to get the Turtlebot to follow a straight path 
#along the wall. The covariance should increase

#INITIALIZE UNCERTAINTY DISTRIBUTION
predicted_state_est = 0
predicted_covariance_est = 0
state_trans_uncertainty_noise = 0
measurement =	0
ave_meas_dist = 0

pub = rospy.Publisher('state_estimate',Config, queue_size =10)



#subscribed to the /scan topic. pointcloud_to_laserscan package converts kinect 3D pcl to 2D laser scan
def get_data(): 
	#make node
 	rospy.init_node('meas_update', anonymous=True)

 	#make subscribers
 
 	#subscriber to /scan data
 	#/scan data publishing frequency: ~8.5-9.25 hz
	rospy.Subscriber('scan', LaserScan, kinect_scan_estimate)
	
	#subscriber to /odom data
	#/odom data publishing frequency: 100 hz
	rospy.Subscriber('odom', Odometry, odom_state_prediction)
	
	# DO ROS HERTz to determine how fast data is comming n
	# this will will determine how fast the filter can be run
	# OR maybe a fixed rate?

	#creating a timed callback to our measurement update function
	# the callback is scheduled for everying 1/1127th of a second (8.8hz)
	rospy.Timer(rospy.Duration(0.1127), meas_update_step, oneshot=False)


	
	#spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	## ?? NEED TIMER TO ENSURE RIGHT COMBO OF DATA  that fres at fixed rate and calls the filtering function at a fixed rate?? ##


#get the state x(t)=[x y th] from odom_data
def odom_state_prediction(odom_data):
	

	global predicted_state_est
	global state_trans_uncertainty_noise
	global predicted_covariance_est

	#get the yaw from the quaternion
	(roll,pitch,yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
	x = odom_data.pose.pose.position.x
	y = odom_data.pose.pose.position.y

	#store odometry predicted state
	predicted_state_est = Config(x,y,yaw)
	#print "predicted state estimate: ",predicted_state_est

	#rospy.loginfo(predicted_state_est)

	# UNCERTAINTY INTRODUCED BY STATE TRANSITION (MEAN = 0, COVARIANCE FOUND PUBLISHED FROM ODOM TOPIC:)
	#predicted_covariance_est = numpy.array([odom_data.pose.covariance[0:6],odom_data.pose.covariance[6:12],odom_data.pose.covariance[12:18],odom_data.pose.covariance[18:24],odom_data.pose.covariance[24:30],odom_data.pose.covariance[30:36]])
	state_trans_uncertainty_noise= numpy.array([[odom_data.pose.covariance[0],odom_data.pose.covariance[1],odom_data.pose.covariance[5]],[odom_data.pose.covariance[6],odom_data.pose.covariance[7],odom_data.pose.covariance[11]], [odom_data.pose.covariance[30],odom_data.pose.covariance[31],odom_data.pose.covariance[35]]])
	#print "state_trans_uncertainty_noise",state_trans_uncertainty_noise
	state_transition_jacobian = numpy.array([[1,0,0],[0,1,0],[0,0,1]])
	#print "state_transition_jacobian", state_transition_jacobian
	predicted_covariance_est = state_transition_jacobian*predicted_covariance_est*numpy.transpose(state_transition_jacobian)+state_trans_uncertainty_noise
	
	#print "predicted covariance estimate: ", predicted_covariance_est

	#return(predicted_state_est) 

#get the measured range
def kinect_scan_estimate(scan_data):
	
	global measurement
	global ave_meas_dist

	#Store laser detected range
	measurement = scan_data.ranges

	sum_dist = 0
	length = 0
	for i in range (310, 330):
		if str(measurement[i]) != 'nan' :
			sum_dist += measurement[i]
			length += 1 

	if length != 0: 
		ave_meas_dist = sum_dist/length
	
	# ELSE REUTURN AN ERROR
	#print ave_meas_dist
	#rospy.loginfo(measurement)
	#print ave_meas_dist

	return(ave_meas_dist, measurement)


#Update the state transition model (prediction) using measured data.
#In order to reduce the uncertainty of the estimated current state.
###
# THIS IS WHERE FILTERING HAPPENS
### 
def meas_update_step(event):
	print "HERE"
	global pub
	global predicted_covariance_est
	#Account for the measurement noise by adding error 
	#meas_noise = ??;


	#expected measurement from odom prediction
	expected_meas = numpy.cross(numpy.array([0, 1, 0]), numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]))
	

	#innovation or measurement residual
	meas_residual = ave_meas_dist - expected_meas

	meas_noise_covariance = 0.005
	#?????????INOVATION COVARIANCE?_IS H CORRECT? NEED TO ADD R_T MEASUREMENT NOISE
	#H = numpy.array([[9999, 0, 0, 0, 0, 0],[0, 9999, 0, 0, 0, 0],[0, 0, 9999, 0, 0, 0],[0, 0, 0, 9999, 0, 0],[0, 0, 0, 0, 9999, 0],[0, 0, 0, 0, 0, 9999]])
	H = numpy.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])


	
	residual_covariance = H*predicted_covariance_est*numpy.transpose(H)+meas_noise_covariance

	#Kalman gain
	near_optimal_kalman_gain = predicted_covariance_est*numpy.transpose(H)*numpy.linalg.inv(residual_covariance)

	updated_state_estimate =  numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]) + numpy.dot(near_optimal_kalman_gain, meas_residual)

	#Updated Covariance estimate
	predicted_covariance_est= (numpy.identity(3) - numpy.dot(near_optimal_kalman_gain,H))*predicted_covariance_est
	#??????????????????????//

	#now that we have updated the state estimate, we must update the odometry data
	# ????????????? rostopic pub /mobile_base/commands/reset_odometry std_msgs/Empty
	# 
	state_estimate = Config(updated_state_estimate[0], updated_state_estimate[1], updated_state_estimate[2])
	print "!!!!!!!!!!!!!!!!!!"
	print "state_estimate", state_estimate

	rospy.logdebug(state_estimate)

	pub.publish(state_estimate)


	fig = plt.figure(1)
	ax = fig.gca()
	plt.axis('equal')
	ax1 = plt.gca()

	x_updated = []
	y_updated = []

	plt.ion()
	plt.show()

	x_updated.append(state_estimate.x)
	y_updated.append(state_estimate.y)

	x_predict = []
	y_predict = []

	x_predict.append(predicted_state_est.x)
	y_predict.append(predicted_state_est.y)

	plt.plot(x_predict, y_predict, 'ro')
	plt.ylabel("odom data")

	plt.plot(x_updated,y_updated,'b*')
	plt.ylabel("estimated state")


	plt.draw()
	plt.grid


if __name__ == '__main__':
	#When program is run, first get the measurements
	try: get_data()
	except rospy.ROSInterruptException: pass