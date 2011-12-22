#!/usr/bin/env python
import csv
import math
import sys
import time

import roslib; roslib.load_manifest('kitchen_data_player')
import rospy
import tf
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose
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
obj_pub = rospy.Publisher('object_poses', PoseArray) # object positions
rospy.init_node('kitchen_player')

posearray = PoseArray()
obj_positions = PoseArray()
posearray.header.frame_id = 'map'
obj_positions.header.frame_id = 'map'

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
             
    # Calculate Human position when standing still and interacting with objects
    if trunk == 'StandingStill' and ((lefthand == 'TakingSomeThing' or lefthand == 'ReleasingGraspOfSomething') or (righthand == 'TakingSomething' or righthand == 'ReleasingGraspOfSomething')):

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

    # Calculate object positions when grasping or releasing objects and calculate orientation of object to human (human orientation shifted by 180 degrees)
    if trunk == 'StandingStill': 
        obj_pose = Pose()
        if (lefthand == 'TakingSomeThing' or lefthand == 'ReleasingGraspOfSomething'): # Object interation with left hand
            obj_pose.position.x = float(row['HALX'])/1000
            obj_pose.position.y = float(row['HALY'])/1000

            # calculate object orientation using object position and human position (without orientation)
            # obj_quat = quaternion_from_euler(0,0,theta + 3.1415)
            deltax = pose.pose.position.x - obj_pose.position.x
            deltay = pose.pose.position.y - obj_pose.position.y
            obj_theta = math.atan2(deltay, deltax)
            obj_quat = quaternion_from_euler(0,0,obj_theta)
            
            obj_pose.orientation.x = float(obj_quat[0])
            obj_pose.orientation.y = float(obj_quat[1])
            obj_pose.orientation.z = float(obj_quat[2])
            obj_pose.orientation.w = float(obj_quat[3])

        if (righthand == 'TakingSomething' or righthand == 'ReleasingGraspOfSomething'): # Object interaction with right hand
            obj_pose.position.x = float(row['HARX'])/1000
            obj_pose.position.y = float(row['HARY'])/1000

            # calculate object orientation using object position and human position (without orientation)
            # obj_quat = quaternion_from_euler(0,0,theta + 3.1415)
            deltax = pose.pose.position.x - obj_pose.position.x
            deltay = pose.pose.position.y - obj_pose.position.y
            obj_theta = math.atan2(deltay,deltax)
            obj_quat = quaternion_from_euler(0,0,obj_theta)

            obj_pose.orientation.x = float(obj_quat[0])
            obj_pose.orientation.y = float(obj_quat[1])
            obj_pose.orientation.z = float(obj_quat[2])
            obj_pose.orientation.w = float(obj_quat[3])


        obj_positions.poses.append(obj_pose)
        obj_pub.publish(obj_positions)

        

