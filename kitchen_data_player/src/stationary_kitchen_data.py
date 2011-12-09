#!/usr/bin/env python
import csv
import math
import sys
import time

import roslib; roslib.load_manifest('kitchen_data_player')
import rospy
import tf
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

# This script parses poses.csv and labels.csv of the TUM kitchen dataset and creates a new csv-file that contains positions, labels
# and timestamps of poses where the human stood still and interacted with objects
# Usage: stationary_kitchen_data.py poses.csv labels.csv output.csv

posesReader = csv.DictReader(open(sys.argv[1], 'rb'), delimiter=',', quotechar='|')
labelsReader = csv.DictReader(open(sys.argv[2], 'rb'), delimiter=',', quotechar='|')

FILE = open(sys.argv[3],"w")
FILE.write("instance,time,BECX,BECY,BECTHETA,lefthand,righthand,trunk\n") 

# Init ROSnode
pose_pub = rospy.Publisher('kitchen_pose', PoseStamped) # current pose
poses_pub = rospy.Publisher('kitchen_poses', PoseArray) # all poses
rospy.init_node('kitchen_player')

posearray = PoseArray()
posearray.header.frame_id = 'map'

for row in posesReader:
    # Write new csv-file with: instance, time, BECX, BEXY, BECTheta, 
    theta = 0
    # Calculate theta
    instance = row['instance']
    x = float(row['SBRX']) - float(row['SBLX'])
    y = float(row['SBRY']) - float(row['SBLY'])
    #theta = math.atan2(-y, x)
    theta = math.atan2(x, -y)
    trunk = 'NONE'
    lefthand = 'NONE'
    righthand = 'NONE'
    for labelrow in labelsReader:
        labelsReader = csv.DictReader(open(sys.argv[2], 'rb'), delimiter=',', quotechar='|')
        if instance == labelrow['instance']:
            lefthand = labelrow['lefthand']
            righthand = labelrow['righthand']
            trunk = labelrow['trunk']
             
    if trunk == 'StandingStill' and ((lefthand != 'CarryingWhileLocomoting' and lefthand != 'Reaching' and lefthand != 'LoweringAnObject') or (righthand != 'CarryingWhileLocomoting' and  righthand != 'Reaching' and righthand != 'LoweringAnObject')):
        FILE.write("%s,%s,%s,%s,%s,%s,%s,%s\n"%(instance, row['time'], row['BECX'], row['BECY'], theta, lefthand, righthand, trunk)) 
        print ("%s,%s,%s,%s,%s,%s,%s,%s"%(instance, row['time'], row['BECX'], row['BECY'], theta, lefthand, righthand, trunk))
        
        # Publish information on ROS-topics on the fly
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(row['BECX'])/1000 # convert to meters here
        pose.pose.position.y = float(row['BECY'])/1000 # convert to meters here
        quat = quaternion_from_euler(0,0,theta)
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        posearray.poses.append(pose.pose)
        
        poses_pub.publish(posearray)
        pose_pub.publish(pose)
        # time.sleep(1/30) # Add this if you want to play in realtime

