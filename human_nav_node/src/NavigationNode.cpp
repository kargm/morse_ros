/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include "ros/ros.h"



#include "human_nav_node/InitWorld.h"
#include "human_nav_node/HANaviPlan.h"
#include "human_nav_node/MotionPlanner.h"

#include "geometry_msgs/Pose.h"


namespace human_nav_node {


MotionPlanner planner;

bool planPath(HANaviPlan::Request &req,
                            HANaviPlan::Response &res) {
//  Pose start = req.start;
//  Pose goal = req.goal;
  return false;
}

bool initWorld(InitWorld::Request &req,
                            InitWorld::Response &res) {
  int result = planner.init(req.pdfilename.c_str(), req.showInterface == 1);
  if (result == 0) {
      return true;
  } else {
      return false;
  }
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

//  NavigationNode *node = new NavigationNode();

  ros::ServiceServer planservice = n.advertiseService("HANaviPlan", human_nav_node::planPath);
  ros::ServiceServer initservice = n.advertiseService("InitWorld", human_nav_node::initWorld);
  ROS_INFO("Ready to plan.");
  ros::spin();

  return 0;
}


