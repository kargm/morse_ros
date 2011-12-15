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

print('Calculating object mean positions:')

table_x_sum = 0
table_y_sum = 0
table_theta_sum = 0
table_counter = 0

drawer_x_sum = 0
drawer_y_sum = 0
drawer_theta_sum = 0
drawer_counter = 0

cupboard_x_sum = 0
cupboard_y_sum = 0
cupboard_theta_sum = 0
cupboard_counter = 0

stove_x_sum = 0
stove_y_sum = 0
stove_theta_sum = 0
stove_counter = 0

for row in objReader:
    # Write new csv-file with: instance, time, BECX, BEXY, BECTheta, 
   
    if (row['location'] == 'table'):
        table_x_sum += float(row['OBJX'])
        table_y_sum += float(row['OBJY'])   
        if float(row['OBJTHETA']) > 0:
            table_theta_sum += float(row['OBJTHETA'])
        else:
            table_theta_sum += float(row['OBJTHETA']) + (2 * math.pi)
        table_counter += 1

    if (row['location'] == 'stove'):
        stove_x_sum += float(row['OBJX'])
        stove_y_sum += float(row['OBJY'])   
        if float(row['OBJTHETA']) > 0:
            stove_theta_sum += float(row['OBJTHETA'])
        else:
            stove_theta_sum += float(row['OBJTHETA']) + (2 * math.pi)
        stove_counter += 1

    if (row['location'] == 'drawer'):
        drawer_x_sum += float(row['OBJX'])
        drawer_y_sum += float(row['OBJY'])   
        if float(row['OBJTHETA']) > 0:
            drawer_theta_sum += float(row['OBJTHETA'])
        else:
            drawer_theta_sum += float(row['OBJTHETA']) + (2 * math.pi)
        drawer_counter += 1

    if (row['location'] == 'cupboard'):
        cupboard_x_sum += float(row['OBJX'])
        cupboard_y_sum += float(row['OBJY'])   
        if float(row['OBJTHETA']) > 0:
            cupboard_theta_sum += float(row['OBJTHETA'])
        else:
            cupboard_theta_sum += float(row['OBJTHETA']) + (2 * math.pi)
        cupboard_counter += 1

table_x_mean = table_x_sum / table_counter
table_y_mean = table_y_sum / table_counter
table_theta_mean = table_theta_sum / table_counter

drawer_x_mean = drawer_x_sum / drawer_counter
drawer_y_mean = drawer_y_sum / drawer_counter
drawer_theta_mean = drawer_theta_sum / drawer_counter

stove_x_mean = stove_x_sum / stove_counter
stove_y_mean = stove_y_sum / stove_counter
stove_theta_mean = stove_theta_sum / stove_counter

cupboard_x_mean = cupboard_x_sum / cupboard_counter
cupboard_y_mean = cupboard_y_sum / cupboard_counter
cupboard_theta_mean = cupboard_theta_sum / cupboard_counter

print('Table mean position: %s, %s, %s'%(table_x_mean, table_y_mean, table_theta_mean))
print('Drawer mean position: %s, %s, %s'%(drawer_x_mean, drawer_y_mean, drawer_theta_mean))
print('Stove mean position: %s, %s, %s'%(stove_x_mean, stove_y_mean, stove_theta_mean))
print('Cupboard mean position: %s, %s, %s'%(cupboard_x_mean, cupboard_y_mean, cupboard_theta_mean))        

        

