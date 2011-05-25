#!/usr/bin/env python  
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('tf')
roslib.load_manifest('roscpp')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('rosgraph_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
import time
import math
import tf
import geometry_msgs
import std_msgs
import nav_msgs
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3Stamped

# Global variables to store the robot starting position in the /map frame for building the /odom frame
odom_init = 0
odom_pose_x = 0
odom_pose_y = 0
odom_pose_z = 0
odom_quat_x = 0
odom_quat_y = 0
odom_quat_z = 0
odom_quat_w = 0

def handle_human_pose(msg, robotname):
    now = rospy.Time.now()
    br2 = tf.TransformBroadcaster()
    br2.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     robotname,
                     "/map")
                     
def handle_map_odom_init(msg, robotname):
    br3 = tf.TransformBroadcaster()

    global odom_init
    global odom_pose_x
    global odom_pose_y
    global odom_pose_z
    global odom_quat_x
    global odom_quat_y
    global odom_quat_z
    global odom_quat_w

    # To obtain the initial point of the odom-frame, the global coordinates are set with the first message
    if odom_init == 0:
        odom_pose_x = msg.pose.pose.position.x
        odom_pose_y = msg.pose.pose.position.y
        odom_pose_z = msg.pose.pose.position.z
        odom_quat_x = msg.pose.pose.orientation.x
        odom_quat_y = msg.pose.pose.orientation.y
        odom_quat_z = msg.pose.pose.orientation.z
        odom_quat_w = msg.pose.pose.orientation.w
        odom_init = 1

    # publish the odom init
    br3.sendTransform((odom_pose_x, odom_pose_y, odom_pose_z),
                    (odom_quat_x, odom_quat_y, odom_quat_z, odom_quat_w),
                    rospy.Time.now(),
                    "/odom",
                    "/map")
    # transformation bewteen map and robot frame
    br3.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/base_footprint",
                     "/map")
                                    
    
    
def handle_odometry(msg, robotname):
    br = tf.TransformBroadcaster()
    now = rospy.Time.now()
    
    global robot_pose_x
    global robot_pose_y
    global robot_pose_z
    global robot_quat_x
    global robot_quat_y
    global robot_quat_z
    global robot_quat_w

    robot_pose_x += robot_pose_x + msg.position.x/100
    robot_pose_y += robot_pose_y + msg.position.y/100
    robot_pose_z += robot_pose_z + msg.position.z/100
    robot_quat_x += msg.orientation.x
    robot_quat_y += msg.orientation.y
    robot_quat_z += msg.orientation.z
    robot_quat_w += msg.orientation.w

    br.sendTransform((robot_pose_x, robot_pose_y, robot_pose_z),
                      (robot_quat_x, robot_quat_y, robot_quat_z, robot_quat_w),
                      rospy.Time.now(),
                      "/base_footprint",
                      "/odom")

if __name__ == '__main__':
    rospy.init_node('morse_tf_broadcaster')
    
    # Initialize odom-frame with robot starting-position
    rospy.Subscriber('/Jido/Pose_sensor',
                     nav_msgs.msg.Odometry,
                     handle_map_odom_init,
                     '/Jido/Pose_sensor')

    # Odometry information frame                      
    #rospy.Subscriber('/Jido/Odometry',
    #                 geometry_msgs.msg.Pose,
    #                 handle_odometry, 
    #                 '/Jido/Odometry')

    # Ground-truth frame for Human Position
    rospy.Subscriber('/Human/GPS',
                     nav_msgs.msg.Odometry,
                     handle_human_pose, 
                     '/human_truepose')

    rospy.spin()
