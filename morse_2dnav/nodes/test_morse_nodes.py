#!/usr/bin/env python 
import roslib
roslib.load_manifest('rosnode')
roslib.load_manifest('rospy')
import rospy
#from rospy import ROSException
from rosnode import rosnode_ping
from rospy import logerr

if __name__ == '__main__':

    if rosnode_ping("morse", 5, 0) == 1:
        pass
    else: 
        logerr("Could not find MORSE-rosnode. Please make sure it is running.")

    if rosnode_ping("morse_tf_broadcaster_navstack", 5, 0) == 1:
        pass
    else: 
        logerr("Could not find MORSE-TF-BROADCASTER. Please make sure it is running.")

    if rosnode_ping("robot_state_publisher", 5, 0) == 1:
        pass
    else: 
        logerr("Could not find ROBOT_STATE_PUBLISHER. Please make sure it is running and the JointStates are published on the correct topic!")
