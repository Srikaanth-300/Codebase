#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
import math
import signal

class nav:
    
    def __init__(self):
        rospy.init_node("autonomous")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.obstacle_threshold = 0.7
        self.distance_to_check = 3.0
        self.max_distance_to_check = 6.0 # unused variable
        self.obstacle_detected = False
        self.rate = rospy.Rate(6)
        self.turned = False       # This variable makes sure the robot doesn't oscillate near the obstacle.
        
        #signal.signal(signal.SIGINT, self.shutdown)  This is unused because the interrupt handling is done on the other script.
        
    def pose_callback(self,msg):     # This is unused as it was a part of laser_scan_tools package testing.
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        
    def laser_callback(self,msg):
        ''' Callback function where the minimum distance and its angle is obtained from scan data and is stored in class variables -obstacle_distance and angle.'''
        self.obstacle_distance = min(msg.ranges)
        i = msg.ranges.index(self.obstacle_distance)
        angle_rad = msg.angle_min + (i * msg.angle_increment)
        self.angle = math.degrees(angle_rad)
        print(self.obstacle_distance, "angle is", self.angle)
        if self.obstacle_distance <= self.obstacle_threshold:
            self.obstacle_detected = True
            #self.obstacle_detect_func()
        else:
            self.obstacle_detected = False
    
    def compute_goal(self):   # Called when the script is run
        '''This function is called once but the while loop inside makes the system run continuously unless shutdown.'''
        while(not rospy.is_shutdown()):
            if(not self.obstacle_detected):                 # Obstacle not detected
                self.exec_func()
            elif(self.obstacle_detected and self.turned):   # Obstacle detected and the rover has rotated, now continue forward until new obstacle detected
                self.exec_func()
            else:                                           # Obstacle detected, now check whether to stop or turn 
                self.obstacle_detect_func()
            self.rate.sleep()
        
    def exec_func(self):  # To publish 0.2 m/s velocity
        '''Publishes 0.2 m/s velocity to move the rover forward.'''
        msg = Twist()
        msg.linear.x = 0.2
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        print("moving")
        self.turned = False

    def stop(self):
        '''Function to stop the rover if the obstacle is lesser than 0.3 m from the lidar, else it doesn't.'''
        if (0 > self.angle >= -90 or 0 <= self.angle <= 90):     # Restricting its FOV to 180 degrees.
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            while(self.obstacle_distance < 0.3):     # While the obstalce is very close keep the rover stopped.
                self.pub.publish(msg)
                rospy.loginfo("Stopping")
        #rospy.loginfo("Stopping")
        #rospy.sleep(3)
        
    def turn(self):
        '''Function to either turn right or left. Class variable "turned" is used to ensure that if the rover has detected an obstacle and turned, keep the rover moving forward until a new obstacle is found (because even after rotating in place the minimum distance to the obstacle stays the same, which would make the rover rotate in place until the detected obstalce is beyond the lidar's FOV. So to avoid this a variable is used which is checked when making a turn if an obstacle is detected).'''
        if (0 > self.angle >= -90):     # Right turn
            turn_msg = Twist()
            turn_msg.linear.x = 0.1
            turn_msg.linear.z = 0.0
            turn_msg.angular.x = 0.0
            turn_msg.angular.y = 0.0
            turn_msg.angular.z = 0.5
            self.pub.publish(turn_msg)
            rospy.loginfo("Turning Right")
            rospy.sleep(3)             # Turn for 3 s
        elif(0 <= self.angle <= 90):   # Left turn
            turn_msg = Twist()
            turn_msg.linear.x = 0.1
            turn_msg.linear.z = 0.0
            turn_msg.angular.x = 0.0
            turn_msg.angular.y = 0.0
            turn_msg.angular.z = -0.5
            self.pub.publish(turn_msg)
            rospy.loginfo("Turning Left")
            rospy.sleep(3)             # Turn for 3 s
        self.turned = True
    def obstacle_detect_func(self):
        '''Once an obstacle is detected, the rover stops only if it's closer than 0.3 m else the member function "turn" is called where the decision is made to turn left or right.'''
        self.stop()
        self.turn()
        
    '''def shutdown(self, signum, frame):   # This is unused and it was a trial to stop this separate node by using signal interrupt.
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)  
        rospy.sleep(1)  
        rospy.signal_shutdown("Shutdown triggered by SIGINT")'''

        
if __name__ == "__main__":
    rospy.sleep(3.0) # Buffer time.
    print("started")
    obj = nav()
    #rate = rospy.Rate(5)
    obj.compute_goal()
    #rate.sleep()
    print("ended")
