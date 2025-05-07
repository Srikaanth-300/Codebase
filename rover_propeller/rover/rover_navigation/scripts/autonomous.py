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
        
    def pose_callback(self,msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        
    def laser_callback(self,msg):
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
        while(not rospy.is_shutdown()):
            if(not self.obstacle_detected):
                self.exec_func()
            elif(self.obstacle_detected and self.turned):
                self.exec_func()
            else:
                self.obstacle_detect_func()
            self.rate.sleep()
        
    def exec_func(self):  # To publish 0.2 m/s velocity
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
        if (0 > self.angle >= -90 or 0 <= self.angle <= 90):
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            while(self.obstacle_distance < 0.3):     # While the obstalce is very close keep the robot stopped.
                self.pub.publish(msg)
                rospy.loginfo("Stopping")
        #rospy.loginfo("Stopping")
        #rospy.sleep(3)
        
    def turn(self):
        
        if (0 > self.angle >= -90):     # Right turn
            turn_msg = Twist()
            turn_msg.linear.x = 0.1
            turn_msg.linear.z = 0.0
            turn_msg.angular.x = 0.0
            turn_msg.angular.y = 0.0
            turn_msg.angular.z = 0.5
            self.pub.publish(turn_msg)
            rospy.loginfo("Turning Right")
            rospy.sleep(3)
        elif(0 <= self.angle <= 90):   # Left turn
            turn_msg = Twist()
            turn_msg.linear.x = 0.1
            turn_msg.linear.z = 0.0
            turn_msg.angular.x = 0.0
            turn_msg.angular.y = 0.0
            turn_msg.angular.z = -0.5
            self.pub.publish(turn_msg)
            rospy.loginfo("Turning Left")
            rospy.sleep(3)
        self.turned = True
    def obstacle_detect_func(self):
        self.stop()
        self.turn()
        
    '''def shutdown(self, signum, frame):
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
    rospy.sleep(3.0)
    print("started")
    obj = nav()
    #rate = rospy.Rate(5)
    obj.compute_goal()
    #rate.sleep()
    print("ended")
