#!/usr/bin/env python
import roslib; roslib.load_manifest('sensor_msgs'); roslib.load_manifest('std_msgs'); 
import rospy
#import time
#import math
import std_msgs
from std_msgs.msg import String
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('/Jido/kuka_base', JointState)
    rospy.init_node('JointState_Publisher')
    i = 0
    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        names = ["seq0", "seq1", "seq2", "seq3", "seq4", "seq5", "seq6"]
        position = [0.15, -1, 0, 1, 0, -1.4, 1.6]
        js = JointState(header, names, position, [], [])
        pub.publish(js)
        rospy.sleep(0.5)
        i = i + 1
        if i > 10:
            break
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
