# **Description:**

The rover has the ability to go in autonomous mode by avoiding obstacles (no extensive path planning) and manual mode by using the ROS-Mobile app. It has a lidar which is used for autonomous navigation.

To launch the system run:
`roslaunch rover_navigation complete_launch.launch`

[Rosserial](http://wiki.ros.org/rosserial) package is required to communicate with the arduino. [rplidar_ros](http://wiki.ros.org/rplidar) is required to interface the lidar data with ROS system (A1 model lidar is used). [Laser_scan_matcher](http://wiki.ros.org/scan_tools) package was also tried but is for future developments. These git submodules weren't changed and used as is.

**_`rover.ino`_** __(rover_navigation/scripts)__ is the arduino code for subscribing to data and giving out PWM values to motor where the pin connections are defined.
The left side motors are coupled together to a single motor driver and the right side motors to another one.

## **Manual operation:**

Once the rover is powered up, connect the ROS-Mobile app with the rover using the same the network. The guide to this setup is given in _OPERATING_MANUAL_ROVER.text_. Once done the rover can be moved around using the joystick.

## **Autonomous mode:**

In autonomous mode the rover moves forward until it detects an obstacle 70 cm from the lidar and turns. The field of view is -45 to 45 (90 degrees) from the center of the lidar. Once the _start_ button _(/start topic)_ is pressed autonomous navigation starts and ends when _stop_ _(/stop topic)_ button is pressed.

_"check"_ node is launched (autonomous_launch.py). Once the start button is pressed, _"autonomous"_ node is launched (autonomous.py) which subscribes to _/scan_ and publishes to _/cmd_vel_ topics running at 6Hz. The minimum and maximum distance to check for obstacles are given in _obstacle_threshold_ and _distance_to_check_.

**Note:** This script assumes only to detect obstacles in front, also it doesn't detect obstacles while its turning (because of sleep function). So obstacles avoidance while the robot is already executing an obstacle avoidance maneuver would not be feasible. That would require reworking the script. Also scan_tools package can be used further to get odometry from laser scan alone. 

All of this is done at statup of the raspberry-pi by using the startup applications.

**Note:** The OS_copy/rover_os contains the configuration files which were dumped in the raspberry pi 4B given to propeller. Those are for backup. If the same thing fails, redump the whole files into SD card. The system used is Ubuntu 20.04 LTS.
