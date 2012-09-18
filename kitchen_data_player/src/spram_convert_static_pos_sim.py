#!/usr/bin/env python  
import sys

import roslib
roslib.load_manifest('kitchen_data_player')
import rospy
import math
import tf
from nav_msgs.msg import Odometry

# This file has been written to convert the simulated data of the SPRAM dataset to csv-files.
# It subscribes to an Odometry message on topic /Human/Pose and writes the positions in which the human did not move much csv-defined in argv[1]

# Global variables
FILE = open(sys.argv[1],"w")
instance = 0
framecounter = 0
last_global_x = 0
last_global_y = 0

def callback(data):

    global instance
    global framecounter
    global last_global_x
    global last_global_y

    now = rospy.Time.now()
    instance = instance + 1
    rospy.sleep(0.04)
    
    global_x = data.pose.pose.position.x
    global_y = data.pose.pose.position.y
    
    z = data.pose.pose.position.z
    rx = data.pose.pose.orientation.x
    ry = data.pose.pose.orientation.y
    rz = data.pose.pose.orientation.z
    rw =  data.pose.pose.orientation.w
    
    frames = 15
    if framecounter%frames == 1:
        last_global_x = global_x
        last_global_y = global_y
            
    if framecounter%frames == 0:
        if last_global_x != 0 and last_global_y != 0:
            delta_x = global_x - last_global_x
            delta_y = global_y - last_global_y
            delta_dist = math.sqrt((delta_x * delta_x) + (delta_y * delta_y))
            velocity = delta_dist * frames

            # If human travelled less than 10 cm, consider him as standing
            # 0.3 seems to not be bad
            if delta_dist < 0.15:
                        #print("STANDING")
                FILE.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n"%(instance,now,global_x,global_y,z,rx,ry,rz,rw))
                print("Human is standing at %s %s %s (time: %s.%s)"%(global_x, global_y, z, now.secs, now.nsecs))
                    # if the human is moving
            else:
                        #print("Moving")
                pass
           
        last_global_x = global_x
        last_global_y = global_y
    framecounter = framecounter +1

if __name__ == '__main__':

    global FILE
    FILE.write("instance,time,BECX,BECY,BECZ,BEC_ROTX,BEC_ROTY,BEC_ROTZ,BEC_ROTW\n") 
    rospy.init_node('spram_converter')
    rospy.Subscriber("/Human/Pose", Odometry, callback)
    rospy.spin()

       
