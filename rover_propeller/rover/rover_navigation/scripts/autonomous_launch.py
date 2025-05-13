#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

process = None  # To keep track of the running node
cmd_vel_pub = None

def stop():    # To ensure the robot stops after the node is killed, zero values are published to /cmd_vel. 
    '''Function to publish zero velocity for a second to stop the rover'''
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.linear.y = 0.0
    stop_msg.linear.z = 0.0
    stop_msg.angular.x = 0.0
    stop_msg.angular.y = 0.0
    stop_msg.angular.z = 0.0
    
    for _ in range(10):
        cmd_vel_pub.publish(stop_msg)
        rospy.sleep(0.1)

def start_clb(msg):
    '''The function starts "autonomous" node by monitoring the global "process" variable. And ensures that only once the node is started after receiving the boolean message from the topic through the app.'''
    global process
    if msg.data:  # If the received message is True
        if process is None:  # Ensure no duplicate nodes are started
            rospy.loginfo("Starting the node...")
            process = subprocess.Popen(["rosrun", "rover_navigation", "autonomous.py"]) 
        else:
            rospy.loginfo("Node is already running!")

def stop_clb(msg):
    '''This function kills the "autonomous" node. Then calls the "stop()" function and resets the "process" variable.'''
    global process
    if msg.data:  # If the received message is True
        if process is not None:
            rospy.loginfo("Stopping the node...")
            process = subprocess.Popen(["rosnode", "kill", "autonomous"])  # Gracefully stop the process
            stop()
            process = None
        else:
            rospy.loginfo("No node is running.")

if __name__ == "__main__":
    rospy.init_node("check")
    rospy.Subscriber("/start", Bool, start_clb)
    rospy.Subscriber("/stop", Bool, stop_clb)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.spin()

