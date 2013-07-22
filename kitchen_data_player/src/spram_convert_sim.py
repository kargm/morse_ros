#!/usr/bin/env python  
import sys

import roslib
roslib.load_manifest('kitchen_data_player')
import rospy

from nav_msgs.msg import Odometry

# This script subscribes to the topic "/Human/Pose" and writes the 2D positions of the human 
# into the csv-file specified by the first argument

# Global variables
FILE = open(sys.argv[1],"w")
instance = 0

def callback(data):
    global instance 
    now = rospy.Time.now()
    instance = instance + 1

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    rx = data.pose.pose.orientation.x
    ry = data.pose.pose.orientation.y
    rz = data.pose.pose.orientation.z
    rw =  data.pose.pose.orientation.w

    FILE.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n"%(instance,now,x,y,z,rx,ry,rz,rw))
    sys.stdout.write(".")
    if instance % 100 == 0:
        print("\n%s"%instance)

if __name__ == '__main__':

    global FILE
    FILE.write("instance,time,BECX,BECY,BECZ,BEC_ROTX,BEC_ROTY,BEC_ROTZ,BEC_ROTW\n") 
    rospy.init_node("spram_sim_converter")
    rospy.Subscriber("/Human/Pose", Odometry, callback)
    rospy.spin()
