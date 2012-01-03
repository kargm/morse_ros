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

for row in objReader:
    # Write new csv-file with: instance, time, BECX, BEXY, BECTheta, 
   
    if (row['location'] == 'table'):
        table_sum += float(row['duration'])
        table_counter += 1

    if (row['location'] == 'stove'):
        stove_sum += float(row['duration'])
        stove_counter += 1

    if (row['location'] == 'drawer'):
        drawer_sum += float(row['duration'])
        drawer_counter += 1

    if (row['location'] == 'cupboard'):
        cupboard_sum += float(row['duration'])
        cupboard_counter += 1

    if (row['location'] == 'navigation'):
        nav_sum += float(row['duration'])
        nav_counter += 1


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
        

