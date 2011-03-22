#!/usr/bin/env python
import roslib;
roslib.load_manifest('human_nav_node')

import sys

import rospy
from human_nav_node.srv import *
from human_nav_node.msg import *

def initWorld(filename, graphics):
    rospy.wait_for_service('InitWorld')
    try:
        proxy = rospy.ServiceProxy('InitWorld', InitWorld)

        resp1 = proxy(filename, graphics)

        return resp1.resultcode
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def initScene(filename):
    rospy.wait_for_service('InitScenario')
    try:
        proxy = rospy.ServiceProxy('InitScenario', InitScenario)
        resp1 = proxy(filename)

        return resp1.resultcode
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def planPath(x1, y1, x2, y2):
    rospy.wait_for_service('HANaviPlan')
    try:
        proxy = rospy.ServiceProxy('HANaviPlan', HANaviPlan)
        start = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x1, y1, 0), None)
        goal = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(x2, y2, 0), None)
        humans = None
        request = NavigationPlanRequest(None, start, goal, humans)
        resp = proxy(request)
        for pose in resp.path.poses:
            print pose
        print "Path has costs " + str(resp.cost)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def changeIFace(showObst, showDist, showVis, shadows):
    rospy.wait_for_service('ChangeInterfaceParams')
    try:
        proxy = rospy.ServiceProxy('ChangeInterfaceParams', ChangeInterfaceParams)
        request = Move3dInterfaceParams(0, 0, None, showObst, showDist, showVis, None, None, None, None, shadows)
        proxy(request)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def changeCamPos(xdest, ydest, zdest, dist, hrot, vrot):
    rospy.wait_for_service('ChangeCamPos')
    try:
        proxy = rospy.ServiceProxy('ChangeCamPos', ChangeCamPos)

        proxy(xdest, ydest, zdest, dist, hrot, vrot)

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
    simpleWorld = roslib.packages.get_pkg_dir("laas_assets") + "/laas-assets/MorseTutorial/simple.p3d"
    initWorld(simpleWorld, 1)
    simpleScene = roslib.packages.get_pkg_dir("laas_assets") + "/laas-assets/MorseTutorial/SCENARIO/simple.sce"
    initScene(simpleScene)
    planPath(-5, -6, 3, 2)
    changeIFace(None, True, None, None)
    changeCamPos(0, 0, 3, 13, -0.4, 0.8)
