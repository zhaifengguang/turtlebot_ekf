# turtlebot_ekf
Tutorial to understand how an ekf is used on a real robot for localization. 

# NOTICE: This is an unfinished tutorial!

To launch the ekf in Gazebo: 

> roslaunch my_tutorial simulation_ekf.launch

To launch on the real Turtlebot: 

> roslaunch my_tutorial ekf.launch

## Overview
Robot kinematics, which is what this tutorial is generally about, has been studied thoroughly in the past decades. 
However, it has almost exculisvely been addressed in deterministic form. Probobalistic robotics generalizes kinematics 
equations to the fact that the outcome of a control is uncertain, due to control noise or unmodeled external effects; which 
is almost certainly always the case when it comes to robot motion in reality. 

This tutorial attempts to understand and simply the complex concepts of robot motion in a very specific setting with the hope of making the 
material easier to grasp. 

## Problem Formulation: 

The tutorial implements an EKF algorithm on a Turtlebot2 robot. 
Two sensors are used: 
  * Turtlebots own odometry data
  * Kinect camera mounted on the turtlebot. 
No additional sensors are being used, such as GPS, external camera, etc. 

The measurements are performed indoors. 

There are no moving objects when data is collected. 

Data is collected in real-time, but there is no real-time demand on reaching a goal position. 


## Preliminaries: 

### Robot Motion

* Kinematics: The calculus describing the effect of conftrol actions on the configuration of a robot.
* Configuration: The configuration of a rigid mobile robot is commobly described by six variables, its 
                 three dimensional Catesian coordinates and its threee Euler angles (roll, pitch, yaw) relative to an external coordinate frame
                 In this tutorial, we are working with the mobile turtlebot operating in a planar environment, so the kinematic state is summarizd 
                 by three variables, referred to as pose.
* Pose: The pose of the robot is described by the 3x1 vector: <x, y, theta>
  * Bearing or heading: The orientation of a robot is often termed bearing or heading
                          In this tutorial, we postulate that a robot with orientation theta = 0  points into the direction of the global x axis and a robt with an 
		                      orientation of theta = pi/2 points in the direction of its y axis.
  * Location: location refers to the robots pose without taking into accounhet orientation, described by the 2D vector: <x,y>
* Kinematic state: The pose and the location of the robot and other objects in the environment may consititute the kinematic state (x_t) of the robot-environment system
* NOTE: x and x_t should not be confused. x refers to a coordinate while x_t is a state that consists of coordinates (x,y,theta)

* Motion model: In probabilistic terms it is the state transition model. In general terms, it is controller that gives a command u to the robot in order to achieve a desired next state. In this tutorial, the control command is a 2x1 vector <v, w> where v is linear velocity and w and angular velocity. 

### Robot Perception: 

* Measurement: Usually sensor readings generate more than a single numerical value. More often than not, the sensor reading consists of an array of values. Depending on the type of sensor data being measured, different mathematical models are used to model the sensor readings: z_t = f(x_t); where f(x_t) depends on the type of data being measured. 
* Sensor error: Every sensor has inaccuracies associated with it. The measurement model is in the form: z_t = f(x_t), a probabilistic density p(zt | xt) with a covariance Q_t.We accound for the error through the covarinace of the probability distribution.   

## Helpful Links and Resources: 

https://vimeo.com/88057157

