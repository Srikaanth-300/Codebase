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

Changing "obstacle_threshold" value will determine the minimum distance to which the robot considers it as an obstacle. The value can be obtained by trying out different values and observing the behavior of the rover (since there is no consideration for the footprint). Greater the value, more easier for rover to turn but it would also make the system more sensitive.

All of this is done at statup of the raspberry-pi by using the startup applications.

### **Future developments:**

There is no map generation, odometery and is more like a wall following robot. So _laser_scan_tools_ can be used to get odometry information from lidar alone which can be used for navigation and to generate maps while navigating. 

**Known bugs:**
- The script (autonomou.py) assumes only to detect obstacles in front, also it doesn't detect obstacles while its turning (because of sleep function). So obstacles avoidance while the robot is already executing an obstacle avoidance maneuver would not be feasible. That would require reworking the script.
- While changing to manual mode from autonomous mode, the node _"autonomous"_ is killed (check autonomous_launch.py) and zero values are published to _cmd_vel_ topic. This worked while testing but had issues while deploying.
- Once the mode is changed to manual, the rover still moved and had to be given zero velocity by pressing the center of the joystick on the app. (This was observed later)


**Note:** The OS_copy/rover_os contains the configuration files which were dumped in the raspberry pi 4B given to propeller. Those are for backup. If the same thing fails, redump the whole files into SD card. The system used is Ubuntu 20.04 LTS.
