#!/usr/bin/env python
import roslib; roslib.load_manifest('morse_bielefeld'); roslib.load_manifest('tf') 
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def human_callback(data):
    # TODO: Calculate yaw
    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    yaw = euler_from_quaternion(quaternion)[2]
    #print("Yaw: %s"%yaw[1])
    print("[%s] Human is at  %s,%s,%s" %(data.header.stamp, data.pose.pose.position.x,data.pose.pose.position.y, yaw))

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
