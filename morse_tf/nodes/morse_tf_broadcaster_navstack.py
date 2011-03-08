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
#import turtlesim.msg
import std_msgs
import nav_msgs
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped

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
    # Hard coded for debugging...
    #br3.sendTransform((5.29036, 0.153143, 0),
    #                 (0.0, 0.0, 0.9915544, 0.129690945),
    #                 rospy.Time.now(),
    #                 "/odom",
    #                 "/map")
    
    br3.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                    (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                    rospy.Time.now(),
                    "/odom",
                    "/map")
    
    #br3.sendTransform((0, 0, 0),
    #                (0, 0, 0, 1),
    #                rospy.Time.now(),
    #                "/odom",
    #                "/map")
                    
def handle_odometry(msg, robotname):
    br = tf.TransformBroadcaster()
    now = rospy.Time.now()
    
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                      (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "/base_footprint",
                      "/odom")
    # The following transformations are for the robot description and should soon be published using a urdf and the robot state publisher
#    br.sendTransform((0.0, 0.0, 0.0), 
#                     (0.0, 0.0, 0.0, 1.0),
#                     rospy.Time.now(),
#                     "/base_link",
#                     "/base_footprint")
                     
#    br.sendTransform((0.38, 0.0, 0.3), 
#                     (0.0, 0.0, 0.0, 1.0),
#                     rospy.Time.now(),
#                     "/base_laser_link",
#                     "/base_link")

if __name__ == '__main__':
    rospy.init_node('morse_tf_broadcaster')
    init = 0
    
    # Initialize odom-frame with robot starting-position
    rospy.Subscriber('/odom_init',
                     nav_msgs.msg.Odometry,
                     handle_map_odom_init,
                     '/odometry_init')

    # Odometry information frame                      
    rospy.Subscriber('/odom',
                     nav_msgs.msg.Odometry,
                     handle_odometry, 
                     '/odometry_information')

    # Ground-truth frame for Human Position
    rospy.Subscriber('/Human/GPS',
                     nav_msgs.msg.Odometry,
                     handle_human_pose, 
                     '/human_truepose')

    rospy.spin()
