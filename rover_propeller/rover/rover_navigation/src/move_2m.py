#!/usr/bin/env python3

# This is a test code to check obstacle avoidance and is not part of the system workflow

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class nav:
    
    def __init__(self):
        rospy.init_node("move_2m")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.obstacle_threshold = 0.35
        
    def laser_callback(self,msg):
        self.obstacle_distance = min(msg.ranges)
        print(self.obstacle_distance)
        if self.obstacle_distance <= self.obstacle_threshold:
            i = msg.ranges.index(self.obstacle_distance)
            angle = msg.angle_min + (i*msg.angle_increment)
            print("The angle of obstacle is: ", angle)
            self.obstacle_detect_func()
        else:
            self.send_commands()
            
    def obstacle_detect_func(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        rospy.loginfo("Stopping")
        self.pub.publish(msg)
        
    def send_commands(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.2
        self.pub.publish(msg)
        rospy.loginfo("Sending_commands")
        
if __name__ == "__main__":
    rospy.sleep(3.0)
    print("started")
    obj = nav()
    rospy.spin()
    print("ended")
