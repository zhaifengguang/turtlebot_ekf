# turtlebot_ekf
Tutorial to understand how an ekf is used on a real robot for localization. 

Turtlebot Intro and Notes
=========================


##Table of Contents <a name="contents"></a>

1. [Description and Capabilities](#Description and Capabilities)

2. [Operating Rules and Instructions](#Operating Rules and Instructions)

  2.1. [Getting started: Software](#Getting started: Software)

  2.2. [Getting started: Hardware](#Getting started: Hardware)

3. [ROS Interface](#ROS Interface)

  3.1. [Running the turtlebot driver](#Running the turtlebot driver)

  3.2. [Teleop the Turtlebot](#Teleop the Turtlebot)

  3.3. [Turtlebot visualisation](#Turtlebot visualisation)

      3.3.1 [Kinect Camera](#Kinect Camera)

  3.4. [Turtlebot simulation](#Turtlebot simulation)

4. [Useful Resources](#Useful Resources)

5. [Cool example projects and demos](#Cool example projects and demos)


## 1. Description and Capabilities <a name="Description and Capabilities"></a>

Visit [WillowGarage](www.willowgarage.com/turtlebot) for an overview of the Turtlebot's specs and capabilities. 
### Turtlebot 2

#### Mobile Base and Power Board
* Kobuki base
* 3000 mA Ni-MH Battery Pack
* 150 degrees/second Signle Axis Gyro
* 12V 1.5Amp Software Enabled Power Supply (for powering the Kinect)

#### Sensor
* Microsoft Kinect
* Kinect Power Board Adaptor Cable

###### [Back to contents](#contents)

## 2. Operating Rules and Instructions <a name="Operating Rules and Instructions"></a>

The turtlebot has a power brick for charging the base. The battery can stay charged for some 
time, but don't let the battery drain before you recharge. 

There is a power switch at the base for powering on the Turtlebot. It will chirp once turned on. 
The status light will blink green. 

Status LED indicates the Kobuki's status: 
  * Green: High battery voltage level
  * Orange: Low batter voltage level (please charge soon)
  * Green blinking: Battery charging
  
link to the Turtlebot Care and feeding: http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/TurtleBot%20Care%20and%20Feeding

### 2.1. Getting started: Software <a name="Getting started: Software"></a>

The [Turtlebot wiki](http://wiki.ros.org/Robots/TurtleBot) instructions are pretty straight forward. 

The [Turtlebot software installation](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) guide is your friend.

There you will find step by step instructions for creating a chained workspace: rocon --> kobuki --> turtlebot.
Or you could simply merge them into one, but I personally perfer the chained workspace to keep files clean and separate. 

###### [Back to contents](#contents)

### 2.2. Getting started: Hardware <a name="Getting started: Hardware"></a>

As long as all the cables are connected properly, there is minimal hardware setup. 
Two USB cables will need to be connected to the operating laptop: 
  1. Kinect USB (make sure it is connected to an SS USB for best sensor information.)
  2. The Turtlebot base USB. 
  
There is a third cable, which connects the Kinect to the Kobuki base for power. 

###### [Back to contents](#contents)


## 3. ROS Interface <a name="ROS Interface"></a>

### 3.1. Running the turtlebot driver <a name="Running the turtlebot driver"></a>

Bringup instruction are found in the following link: http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/TurtleBot%20Bringup

To make sure the Kobuki base starts up, you must ensure that your system has the udev
rule applied for /dev/kobuki.

> ls -n /dev | grep kobuki

To start up the driver, in the sourced ROS environment: 

> roslaunch turtlebot_bringup minimal.launch


###### [Back to contents](#contents)


### 3.2. Teleop the Turtlebot <a name="Teleop the Turtlebot"></a>

There are many ways the Turtlebot can be teleoperated: 

  * [Keyboard](http://wiki.ros.org/turtlebot_teleop/Tutorials/indigo/Keyboard%20Teleop)
  * [Joystick](http://wiki.ros.org/turtlebot_teleop/Tutorials/indigo/Joystick%20Teleop)
      * I would recommend usign the [sixaxis](https://help.ubuntu.com/community/Sixaxis) Ubuntu package to connect to a PS3 controller instead of the ps3joy ROS package. 
  * [Qt](http://wiki.ros.org/rocon_qt_teleop/Tutorials/indigo/Qt%20Teleop%20a%20turtlebot)
  * [Interactive Markers](http://wiki.ros.org/turtlebot_interactive_markers/Tutorials/indigo/UsingTurtlebotInteractiveMarkers)


###### [Back to contents](#contents)


### 3.3. Turtlebot visualisation <a name="Turtlebot visualisation"></a>

To start the kinect camera, first ensure that the [minimal software](http://wiki.ros.org/turtlebot_bringup/Tutorials/indigo/TurtleBot%20Bringup) (minimal.launch) has been launched on the robot. Also ensure the kinect is plugged into both the laptop and the Turtlebot. 

> roslaunch turtlebot_bringup 3dsensor.launch

3D visualization linke: http://wiki.ros.org/turtlebot/Tutorials/indigo/3D%20Visualisation

###### [Back to contents](#contents)


#### 3.3.1 Kinect Camera <a name="Kinect Camera"></a>


###### [Back to contents](#contents)


### 3.4. Turtlebot simulation <a name="Turtlebot simulation"></a>


###### [Back to contents](#contents)


## 4. Useful Resources <a name="Useful Resources"></a>


###### [Back to contents](#contents)


## 5. Cool example projects and demos <a name="Cool example projects and demos"></a>


###### [Back to contents](#contents)


Author: Mahdieh Nejati 

Created: Feburary 2015


