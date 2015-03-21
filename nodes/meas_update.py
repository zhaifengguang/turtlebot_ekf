#!/usr/bin/env python

# MEASUREMENT NODE ___ THE MEASUREMENT MODEL OF THE LOCALISATION ALGORITHM 
# Extended Kalman Filtering and estimation happens here. 
#
# This is a crude first iteration. 
# Through writing this code I am understanding how a Bayesian filter, specifically in 
# the form of the EKF is implemented in practice on a real robot. 
# Through the process of learning I will develop a tutorial that is easy to understand and follow for my own and hopefully
# others reference. 
# Then I will implement a better EKF using a more accurate measurement model and prediction update step, through 
# using more of the Kinect data, using Tags as landmarks, and a better mathematical model of the measurement. 
#
# Subscribed to: /odom
#				 /scan
#
# Publishes to:  /state_estimate
#
#
# Author: Mahdieh Nejati Javaremi 
# m.nejati@u.northwestern.edu
# Winter 2015


# Importing necessary libraries and populating namespaces: 
import rospy
import sys
import numpy 
import std_msgs.msg
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from my_tutorial.msg import * 
from tf.transformations import euler_from_quaternion
from math import sqrt
from matplotlib.patches import Ellipse



# FILTER INITIALIZATION 
# PRIOR KNOWLEDGE OF STATE
predicted_state_est = 0
predicted_covariance_est = 0
state_trans_uncertainty_noise = 0
measurement =	0
ave_meas_dist = 0

# Create instance of publisher
pub = rospy.Publisher('state_estimate',Config, queue_size =10)


# First step is to get all the required information. 
def get_data(): 

	#Initialize node
 	rospy.init_node('meas_update', anonymous=True)

 	
 	# Initiate subscriber to /scan topic. 
 	# pointcloud_to_laserscan package converts kinect 3D pcl to 2D laser scan
 	# /scan data publishing frequency: ~8.5-9.25 hz
	rospy.Subscriber('scan', LaserScan, kinect_scan_estimate)
	
	# Initiate subscriber to /odom topic
	# /odom data publishing frequency: 100 hz
	rospy.Subscriber('odom', Odometry, odom_state_prediction)
	
	# Did 'ros hertz' to determine how fast data is being published and subsequently recieved. 
	# Data publishing rate will determine how fast the filter can be run, 
	# Note to self: ...Or maybe it's just fine at a fixed rate?

	# Create a timed callback to the measurement update function. 
	# The callback is scheduled for every 1/1127 th of a second (8.8hz)
	rospy.Timer(rospy.Duration(0.1127), meas_update_step, oneshot=False)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	# The timer rate for now seems to ensure that the correct combination of scan data and odometer data are being sent to the
	# filter. 
	

#get the predicted state x(t)=[x y th] from odom_data
def odom_state_prediction(odom_data):
	

	global predicted_state_est
	global state_trans_uncertainty_noise
	global predicted_covariance_est

	# Get the yaw (theta) from the quaternion using the tf.transformations euler_from_quaternion function. 
	(roll,pitch,yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
	x = odom_data.pose.pose.position.x
	y = odom_data.pose.pose.position.y

	# Package desired state in a custom message of type 'Config' 
	predicted_state_est = Config(x,y,yaw)

	#rospy.loginfo(predicted_state_est)

	# UNCERTAINTY INTRODUCED BY STATE TRANSITION (MEAN = 0, COVARIANCE PUBLISHED BY ODOM TOPIC: )
	# Odom covariance matrix is 6 x 6. We need a 3 x 3 covariance matrix of x, y and theta. Omit z, roll and pitch data. 
	state_trans_uncertainty_noise= numpy.array([[odom_data.pose.covariance[0],odom_data.pose.covariance[1],odom_data.pose.covariance[5]],[odom_data.pose.covariance[6],odom_data.pose.covariance[7],odom_data.pose.covariance[11]], [odom_data.pose.covariance[30],odom_data.pose.covariance[31],odom_data.pose.covariance[35]]])
	
	# Define the state transition jacobian. 
	state_transition_jacobian = numpy.array([[1,0,0],[0,1,0],[0,0,1]])
	
	# Calculated the total uncertainty of the predicted state estimate
	predicted_covariance_est = state_transition_jacobian*predicted_covariance_est*numpy.transpose(state_transition_jacobian)+state_trans_uncertainty_noise
	
	
# Get the measured range
# Simplest measurement model using scan data. 
# We are throwing most of the data away for the sake of simplicity. 
# Having more measurement data will reduce uncertainty.
# if the PCL was used instead of laser scan, the measurement update model would PERHAPS reduce the uncertainty more...  
def kinect_scan_estimate(scan_data):
	
	global measurement
	global ave_meas_dist


	measurement = scan_data.ranges
	
	# Average measured range/distance of 20 samples from sample 310 to 330 (out of a total 640 samples)
	sum_dist = 0
	length = 0
	#for i in range (310, 330):
	for i in range (580, 600):
		if str(measurement[i]) != 'nan' :
			sum_dist += measurement[i]
			length += 1 

	if length != 0: 
		ave_meas_dist = sum_dist/length
	
	# ELSE REUTURN AN ERROR
	
	#rospy.loginfo(measurement)


#Update the prediction using measured data to reduce the uncertainty of the estimated current state.

###
# THIS IS WHERE FILTERING HAPPENS
### 

# In the extended kalman filter, the observation(measurement) model
# does not need to be a linear function of state, but may instead be 
# a differentiable function. I currently have a linear model, similar to 
# a Kalman filter.
def meas_update_step(event):
	
	global pub
	global predicted_covariance_est
	
 	# Calculating the measurement we expect to get based on received odometery data. 
	expected_meas = numpy.cross(numpy.array([0, 1, 0]), numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]))
	
	#innovation or measurement residual: The difference between our actual measurement and the measurement we expected to get. 
	meas_residual = ave_meas_dist - expected_meas

	#Account for the measurement noise by adding error 
	meas_noise_covariance = 0.005

	# Measurement jacobian: 
	H = numpy.array([[9999, 0 , 0],[0, 1, 0],[0 , 0, 9999]])

	# Innovation (or residual) covariance
	residual_covariance = H*predicted_covariance_est*numpy.transpose(H)+meas_noise_covariance

	# Near-optimal Kalman gain
	kalman_gain = predicted_covariance_est*numpy.transpose(H)*numpy.linalg.inv(residual_covariance)

	# Updated state estimate
	updated_state_estimate =  numpy.array([predicted_state_est.x, predicted_state_est.y, predicted_state_est.th]) + numpy.dot(kalman_gain, meas_residual)

	# Storing predicted covariance for plot
	pre_cov_store = predicted_covariance_est

	# Updated covariance estimate
	predicted_covariance_est= (numpy.identity(3) - numpy.cross(kalman_gain,H))*predicted_covariance_est
	
	#storing updated covariance estimate for plotting
	up_cov_store = predicted_covariance_est

	# Now that we have updated the state estimate, we must update prediction step to use this data for 
	# next iteration. 
	# Tried using: rostopic pub /mobile_base/commands/reset_odometry std_msgs/Empty 
	# That's not the way to do it as it only resets to zero... figure this out.  

	# Package the state estimate in custom message of type 'Config'
	state_estimate = Config(updated_state_estimate[0], updated_state_estimate[1], updated_state_estimate[2])
	
	rospy.logdebug(state_estimate)

	pub.publish(state_estimate)


	# Plotting the predicted and updated state estimates as well as the uncertainty ellipse to see if 
	# filter is behaving as expected. 

	fig = plt.figure(1)
	ax = fig.gca()
	plt.axis('equal')
	ax1 = plt.gca()

	# Updated state estimate: 
	x_updated = []
	y_updated = []

	plt.ion()
	plt.show()

	x_updated.append(state_estimate.x)
	y_updated.append(state_estimate.y)

	# Update is plotted as blue points. 
	plt.plot(x_updated,y_updated,'b*')
	plt.ylabel("y")
	plt.xlabel("x")

	# Predicted state estimate: 
	x_predict = []
	y_predict = []

	x_predict.append(predicted_state_est.x)
	y_predict.append(predicted_state_est.y)

	# Prediction is plotted as red points. 
	plt.plot(x_predict, y_predict, 'ro')
	plt.ylabel("y")
	plt.xlabel("x")


	# Plot the covariance
	# I expect the updated covariance to decrease in the direction of measurement and increase in the 
	# direction that I am not taking any measurements.  

	lambda_pre,v=numpy.linalg.eig(pre_cov_store)
	lambda_pre = numpy.sqrt(lambda_pre)

	ax = plt.subplot(111, aspect = 'equal')

	for j in xrange(1,4):
		ell = Ellipse(xy=(numpy.mean(x_predict),numpy.mean(y_predict)), width=lambda_pre[0]/(j*19), height=lambda_pre[1]/(j*10),angle=numpy.rad2deg(numpy.arccos(v[0,0])))

	ell.set_facecolor('none')
	ax.add_artist(ell)

	lambda_up,v=numpy.linalg.eig(up_cov_store)
	lambda_up= numpy.sqrt(lambda_up)

	ax = plt.subplot(111, aspect = 'equal')

	for j in xrange(1,4):
		ell = Ellipse(xy=(numpy.mean(x_updated),numpy.mean(y_updated)), width=lambda_up[0]/j*10, height=lambda_up[1]/j*10,angle=numpy.rad2deg(numpy.arccos(v[0,0])))
	ell.set_facecolor('none')
	ax.add_artist(ell)

	plt.show()
	plt.draw()
	plt.grid


if __name__ == '__main__':
	#When program is run, first get the measurements
	try: get_data()
	except rospy.ROSInterruptException: pass
