#!/usr/bin/env python
import roslib; roslib.load_manifest('morse_bielefeld'); roslib.load_manifest('tf') 
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


def human_callback(data):
    # TODO: Calculate yaw
    quaternion = 
    yaw = euler_from_quaternion(data.pose.pose.orientation)
    print("Yaw: %s"%yaw[1])
    print("[%s] Human is at  %s" %(data.header.stamp, data.pose.pose.position.x,data.pose.pose.position.y))

def robot_callback(data):
    print("Robot is at  %s" % data.pose.pose.position.x)

def human_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/Human/Human_pose", Odometry, human_callback)
    rospy.spin()

def robot_listener():
    rospy.Subscriber("/Jido/Robot_pose", Odometry, robot_callback)
    rospy.spin()

if __name__ == '__main__':
    human_listener()
    #robot_listener()
