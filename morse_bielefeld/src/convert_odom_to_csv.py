#!/usr/bin/env python
import roslib; roslib.load_manifest('morse_bielefeld'); roslib.load_manifest('tf') 
import rospy
import sys
import message_filters
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from approxsync import ApproximateSynchronizer

#out_file = open(sys.argv[1],"w")
#out_file.write("time,human_x,human_y,human_yaw,robot_x,robot_y,robot_yaw\n")
human_topic = "/Human/Human_pose"
robot_topic = "/Jido/Robot_pose"

def callback(human_data, robot_data):
    # TODO: Calculate yaw
    human_quaternion = [human_data.pose.pose.orientation.x, human_data.pose.pose.orientation.y, human_data.pose.pose.orientation.z, human_data.pose.pose.orientation.w]
    robot_quaternion = [robot_data.pose.pose.orientation.x, robot_data.pose.pose.orientation.y, robot_data.pose.pose.orientation.z, robot_data.pose.pose.orientation.w]

    human_yaw = euler_from_quaternion(human_quaternion)[2]
    robot_yaw = euler_from_quaternion(robot_quaternion)[2]

    print("%s,%s,%s,%s,%s,%s,%s" %(human_data.header.stamp, human_data.pose.pose.position.x,human_data.pose.pose.position.y, human_yaw,robot_data.pose.pose.position.x,robot_data.pose.pose.position.y, robot_yaw))

def time_synchronizer():
    rospy.init_node('synchronizer', anonymous=True)

    #sync = message_filters.TimeSynchronizer 
    sync = ApproximateSynchronizer

    human_sub = message_filters.Subscriber(human_topic, Odometry)
    robot_sub = message_filters.Subscriber(robot_topic, Odometry)
    ts = sync(0.016, [human_sub, robot_sub], 1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    print("time,human_x,human_y,human_yaw,robot_x,robot_y,robot_yaw")
    time_synchronizer()

