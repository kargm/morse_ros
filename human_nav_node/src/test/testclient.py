#!/usr/bin/env python
import roslib;
roslib.load_manifest('human_nav_node')

import sys

import rospy
from human_nav_node.srv import *
from human_nav_node.msg import *

def initWorld():
    rospy.wait_for_service('InitWorld')
    try:
        proxy = rospy.ServiceProxy('InitWorld', InitWorld)
        testpath = roslib.packages.get_pkg_dir("laas_assets") + "/laas-assets/MorseTutorial/empty.p3d"
        resp1 = proxy(testpath, 1)

        return resp1.resultcode
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def planPath(x1, y1, x2, y2):
    rospy.wait_for_service('HANaviPlan')
    try:
        proxy = rospy.ServiceProxy('HANaviPlan', HANaviPlan)
        start = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(1,2, 0), None)
        goal = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(4,4, 0), None)
        humans = None
        request = NavigationPlanRequest(None, start, goal, humans)
        resp = proxy(request)
        for pose in resp.path.poses:
            print pose
        print "Path has costs " + str(resp.cost)
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
    planPath(1, 2, 3, 4)
