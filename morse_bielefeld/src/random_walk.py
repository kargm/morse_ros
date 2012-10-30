#! /usr/bin/env python

import roslib; roslib.load_manifest('morse_bielefeld')
import rospy
from morsetesting.msg import *
from geometry_msgs.msg import *

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
#import actionlib_tutorials.msg

def start_position():
    client = actionlib.SimpleActionClient('Motion_Controller/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal(Pose(Point(0,-3,0.0), Quaternion(0.0,0.0,0.0,1.0)))
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    status = client.send_goal_and_wait(goal, rospy.Duration(5))
    print("Got this status: " + str(status))
    client.wait_for_result()
    return client.get_result()

def cupboard_1():
    client = actionlib.SimpleActionClient('Motion_Controller/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal(Pose(Point(2,2,0.0), Quaternion(0.0,0.0,0.0,1.0)))
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    status = client.send_goal_and_wait(goal, rospy.Duration(5))
    print("Got this status: " + str(status))
    client.wait_for_result()
    return client.get_result()

def cupboard_2():
    client = actionlib.SimpleActionClient('Motion_Controller/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal(Pose(Point(2,0,0.0), Quaternion(0.0,0.0,0.0,1.0)))
    print("Sending the robot to cupboard 2...(timeout=5sec)")
    status = client.send_goal_and_wait(goal, rospy.Duration(5))
    print("Got this status: " + str(status))
    client.wait_for_result()
    return client.get_result()



if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('random_walk_client')
        result = cupboard_1()
        print("Reached CB1: s" + str(result))
        result = start_position()
        print("Reached Start: s" + str(result))
        result = cupboard_2()
        print("Reached CB2: s" + str(result))

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
