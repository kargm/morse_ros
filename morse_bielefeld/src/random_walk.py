#! /usr/bin/env python

import roslib; roslib.load_manifest('morse_bielefeld')
import rospy
#from morsetesting.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
#import actionlib_tutorials.msg

def start_position():
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = -3
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0;
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def cupboard_1():
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 2
    goal.target_pose.pose.position.y = 2
    goal.target_pose.pose.orientation.w = 1.0;
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def cupboard_2():
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 2
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0;
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('random_walk_client')

        result = cupboard_1()
        print("Reached CB1: " + str(result))
        result = start_position()
        print("Reached Start: " + str(result))
        result = cupboard_2()
        print("Reached CB2: " + str(result))

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
