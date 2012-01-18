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

# Usage: calculate_obj_mean_positinos.py clustered_obj_positions.csv

objReader = csv.DictReader(open(sys.argv[1], 'rb'), delimiter=',', quotechar='|')

print('Calculating mean durations:')

table_sum = 0
table_counter = 0

drawer_sum = 0
drawer_counter = 0

cupboard_sum = 0
cupboard_counter = 0

stove_sum = 0
stove_counter = 0

nav_sum = 0
nav_counter = 0

# write different files of locations for creating gaussians over time
TABLE_FILE = open("table.csv","w")
STOVE_FILE = open("stove.csv","w")
DRAWER_FILE = open("drawer.csv","w")
CUPBOARD_FILE = open("cupboard.csv","w")
NAV_FILE = open("navigation.csv","w")
TABLE_FILE.write("instance,location,duration\n")
STOVE_FILE.write("instance,location,duration\n")
DRAWER_FILE.write("instance,location,duration\n")
CUPBOARD_FILE.write("instance,location,duration\n")
NAV_FILE.write("instance,location,duration\n")

for row in objReader:
    # Write new csv-file with: instance, time, BECX, BEXY, BECTheta, 
   
    if (row['location'] == 'table'):
        table_sum += float(row['duration'])
        table_counter += 1
        TABLE_FILE.write("%s,%s,%s\n"%(row['instance'], row['location'], row['duration']))

    if (row['location'] == 'stove'):
        stove_sum += float(row['duration'])
        stove_counter += 1
        STOVE_FILE.write("%s,%s,%s\n"%(row['instance'], row['location'], row['duration']))

    if (row['location'] == 'drawer'):
        drawer_sum += float(row['duration'])
        drawer_counter += 1
        DRAWER_FILE.write("%s,%s,%s\n"%(row['instance'], row['location'], row['duration']))

    if (row['location'] == 'cupboard'):
        cupboard_sum += float(row['duration'])
        cupboard_counter += 1
        CUPBOARD_FILE.write("%s,%s,%s\n"%(row['instance'], row['location'], row['duration']))

    if (row['location'] == 'navigation'):
        nav_sum += float(row['duration'])
        nav_counter += 1
        NAV_FILE.write("%s,%s,%s\n"%(row['instance'], row['location'], row['duration']))


table_mean = table_sum / table_counter

drawer_mean = drawer_sum / drawer_counter

stove_mean = stove_sum / stove_counter

cupboard_mean = cupboard_sum / cupboard_counter

nav_mean = nav_sum / nav_counter

print('Table mean duration: %s s'%table_mean)
print('Drawer mean duration: %s s'%drawer_mean)
print('Stove mean duration: %s s'%stove_mean)
print('Cupboard mean duration: %s s'%cupboard_mean)        
print('Navigation mean duration: %s s'%nav_mean)
        

