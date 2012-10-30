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

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.

    client = actionlib.SimpleActionClient('Motion_Controller/move_base', MoveBaseAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.

    goal = MoveBaseGoal(Pose(Point(2,0,0.0), Quaternion(0.0,0.0,0.0,1.0)))
    # Sends the goal to the action server.
    #client.send_goal(goal)
    print("Sending a first goal to the robot...(timeout=5sec)")
    status = client.send_goal_and_wait(goal, rospy.Duration(5))

    print("Got this status: " + str(status))
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result: s" + str(result))
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
