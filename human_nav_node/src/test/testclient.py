#!/usr/bin/env python
import roslib;
roslib.load_manifest('human_nav_node')

import sys

import rospy
from human_nav_node.srv import *

def initWorld():
    rospy.wait_for_service('InitWorld')
    try:
        proxy = rospy.ServiceProxy('InitWorld', InitWorld)
        testpath = roslib.packages.get_pkg_dir("laas_assets") + "/laas-assets/MorseTutorial/empty.p3d"
        resp1 = proxy(testpath, 0)
        return resp1.resultcode
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# def usage():
#     return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 3:
    #     x = int(sys.argv[1])
    #     y = int(sys.argv[2])
    # else:
    #     print usage()
    #     sys.exit(1)

    # print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
    initWorld()
