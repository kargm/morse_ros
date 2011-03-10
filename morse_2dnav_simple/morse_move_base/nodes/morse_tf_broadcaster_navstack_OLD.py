#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
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
                     
def handle_jido_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    now = rospy.Time.now()

    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                      (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "/odom",
                      "/map")
                     
    br.sendTransform((0.0, 0.0, 0.0), 
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "/base_footprint",
                     "/odom")
                     
    br.sendTransform((0.0, 0.0, 0.0), 
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "/base_link",
                     "/base_footprint")
                     
    br.sendTransform((0.0, 0.0, 0.0), 
                     (0.0, 0.0, 0.0, 1.0),
                     rospy.Time.now(),
                     "/base_laser_link",
                     "/base_link")

if __name__ == '__main__':
    rospy.init_node('morse_tf_broadcaster')
    
    rospy.Subscriber('/Human/GPS',
                     nav_msgs.msg.Odometry,
                     handle_human_pose,
                     '/HumanGPS')

    # should sbscribe to /base_pose_ground_truth to build tf-tree correctly
    # subscribe to /odom to set odom as reference frame for debugging                     
    rospy.Subscriber('/base_pose_ground_truth',
                     nav_msgs.msg.Odometry,
                     handle_jido_pose,
                     '/odom')
    
    rospy.spin()
