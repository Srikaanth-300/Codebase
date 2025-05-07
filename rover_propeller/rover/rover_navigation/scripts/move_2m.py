#!/usr/bin/env python3

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
        self.distance_to_check = 3.0
        self.max_distance_to_check = 6.0
        
    def laser_callback(self,msg):
        self.obstacle_distance = min(msg.ranges)
        print(self.obstacle_distance)
        if self.obstacle_distance <= self.obstacle_threshold:
            self.obstacle_detect_func()
            self.new_goal(msg)
        else:
            self.send_commands()
            
    def new_goal(self, msg):
        dist = []
        x = msg.ranges.index(self.obstacle_distance)
        for i in range(len(msg.ranges)):
        
            d = msg.ranges[i]
            if (d - self.obstacle_distance < 0.001) or (d - self.obstacle_distance < -0.001):
                continue
            if (d >=self.distance_to_check and d <=self.max_distance_to_check):
                dist.append(d)
                #angle = msg.angle_min +(i*msg.angle_increment)
                #print("The new goal point chosen is: ", math.degrees(angle),"also the d value is: ",d)
                #break
            else:
                continue
        point = max(dist)
        x = msg.ranges.index(point)
        print("The goal angle chosen is :",  math.degrees(msg.angle_min +(x*msg.angle_increment))," and the distance is: ",point)
    
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
