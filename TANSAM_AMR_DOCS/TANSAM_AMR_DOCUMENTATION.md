# **Connections:**
-	A single button to power the whole system and a separate button to power the motor (which must be kept off until goals are given).
-	Intel realsense, RPLidar and the Arduino Mega are connected to NUC via USB ports.
-	Encoder pins of the left motor are connected to 18th and 19th pin of Arduino for tick count and wheel direction respectively (ticks published on l_wheel topic).
-	Encoder pins of the right motor are connected to 20th and 21th pin of Arduino for tick count and wheel direction respectively (ticks published on r_wheel topic).
-	PWM and DIR for right motor is connected to 7th and 6th pins respectively.
-	PWM and DIR for left motor is connected to 4th and 3th pins respectively.

# **To run the system:**
-	Power the robot which automatically starts the system else type `roslaunch amr_launch amr.launch` in the terminal.
-	Ensure the motor kill switch is turned on before running the system.
-	Then connect the ROS app with the master running in NUC by mentioning the correct ip address and port of the master (check by using ipconfig in a new terminal).
-	Once connected move to the VIZ tab in the app where the controls can be found.
-	At the robot’s home position, press the Pose reset button to give an initial estimate for localization in autonomous mode.
-	The required goals can be sent by pressing the respective button and the plan can be viewed in the display.
-	For manual operation, use the joystick in the VIZ tab to move around.

# **Complete flow of the stack:**
-	All the launch files are inside _catkin_ws_ folder. The command to run the navigation system is `roslaunch amr_launch amr.launch` which can be found in _amr_launch/launch_ directory.
-	This starts the rosserial_arduino node, rplidar node, navigation and display launch.
-	In the same directory _imu.launch_ runs the scripts for getting IMU data (through camera) and communication between the ROS app and the master.
-	The script files are in _amr_launch/scripts_ directory.
-	_new_goal3.py_ is where the positions of respective goals (seen in the app) is given. The script is executed when goal points are pressed in the app.
-	_pub_initpose.py_ is where the initial pose of the robot (the starting point) is given. This script is executed when the ‘Intial Pose’ button is pressed.
-	_imu.py_ is where the message (imu type) is published, subscribing to data from the camera.
-	_tansam_amr2_description_ folder contains the urdf, controller, rviz launch commands and the robot_localization launch file for fusing odom and imu data.
-	_rur_navigation_ folder handles the launch of move_base and amcl. _rur_navigation2.launch_ includes the above specified files and also the map is saved outside the workspace, i.e. at home.

**Note:** Make sure to give proper initial_pose values in the script if remapping is done. And ensure the robot's localization (amcl) and the map aligns properly before giving goals. If new places are added as goals in the app, add those in the script with respective numbers.

# **To remap:**
-	The environment is mapped by using gmapping package from ROS. 
-	In order to remap the environment run `roslaunch gmapping slam_gmapping_pr2.launch` which can be found in _gmapping/launch_ directory.
-	After launching gmapping node, move the robot using the app.
-	Once all the area is mapped then save the generated map by running `rosrun map_server map_saver –f [your filename]`
-	This will save a pgm and yaml file with the specified filename. 
-	Then give the path of the newly saved map to the parameter _map_file_ in _rur_navigation2.launch_.
# **ROS app:**
-	All the required widgets are configured by specifying the exact topic name and the message type.
-	The goal points can be given by pressing the buttons.
-	Ensure that the master is running before trying to connect.
