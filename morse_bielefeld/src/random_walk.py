#! /usr/bin/env python

import roslib; roslib.load_manifest('morse_bielefeld')
import rospy
#from morsetesting.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
# Brings in the SimpleActionClient
import actionlib

from random import randint

def start_position():
    print("Going to starting position...")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = -3
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0;
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def cupboard_1():
    print("Going to cupboard 1")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 2
    goal.target_pose.pose.position.y = 2
    goal.target_pose.pose.orientation.w = 1.0;
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def cupboard_2():
    print("Going to cupboard 2")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 2
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0;
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def cupboard_3():
    print("Going to cupboard 3")
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = 2
    goal.target_pose.pose.position.y = -3
    goal.target_pose.pose.orientation.w = 1.0;
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

# Walk around between randomly chosen cupboard and starting position
def go_to_random_cupboard_and_back():
    rnd = randint(1,3)
    print("Selected Cupboard %s\n"%rnd)
    if rnd == 1:
        cupboard_1()
    elif rnd == 2:
        cupboard_2()
    elif rnd == 3:
        cupboard_3()
    start_position()
    return 0

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('random_walk_client')
        i = 0
        while i < 100:
            go_to_random_cupboard_and_back()
            i = i + 1

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
