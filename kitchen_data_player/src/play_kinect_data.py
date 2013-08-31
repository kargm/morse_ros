#!/usr/bin/env python
import csv
import math
import sys
import time

import roslib; roslib.load_manifest('kitchen_data_player');  roslib.load_manifest('nav_msgs')
import rospy
import tf
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


# This script uses csv-files from Kinect data recorded with the Windows Kinect SDK and publishes
# the human 2D poses as nav_msgs/Odometry on the topic /Human/Pose

def publish_kinect():
    kinect_x = 1.3175
    kinect_y = 0.0946

    br.sendTransform((kinect_x, kinect_y, 0),
                     quaternion_from_euler(0,0,1.57075) ,
                     rospy.Time.now(),
                     "kinect",
                     "map")

posesReader = csv.DictReader(open(sys.argv[1], 'rb'), delimiter=',', quotechar='|')
# Init ROSnode
pose_pub = rospy.Publisher('/Human/Pose', Odometry)             # current pose

rospy.init_node('kitchen_player')

br = tf.TransformBroadcaster()
tf_listener = tf.TransformListener()
global_x = 0
global_y = 0

for row in posesReader:
    publish_kinect()

    #TODO: include and calculate orientation
    theta = 0

    quat = quaternion_from_euler(0,0,0)
    br.sendTransform((float(row['z']), float(row['x']), 0),
                     quat,
                     rospy.Time.now(),
                     "human_pose",
                     "kinect")

    global_x = 0
    global_y = 0

    # get human position in reference to map
    try:
        now = rospy.Time.now() - rospy.Duration(0.0)
        tf_listener.waitForTransform("map", "human_pose", rospy.Time(0), rospy.Duration(0.001))
        (human_trans, human_rot) = tf_listener.lookupTransform("map", "human_pose", rospy.Time(0))
        global_x = human_trans[0]
        global_y = human_trans[1]
        print("humantrans: x: %s, y: %s"%(human_trans[0], human_trans[1]))

    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print('.')

    human_pose = Odometry()
    human_pose.header.stamp = now
    human_pose.pose.pose.position.x = float(global_x)
    human_pose.pose.pose.position.y = float(global_y)
    human_pose.pose.pose.orientation = Quaternion(*quat)
    pose_pub.publish(human_pose)
    
    time.sleep(0.033) #realtime
    
