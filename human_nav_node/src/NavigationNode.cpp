/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include "ros/ros.h"

#include <mhpm3d.h>

#include "human_nav_node/InitWorld.h"
#include "human_nav_node/HANaviPlan.h"
#include "geometry_msgs/Pose.h"


hri_bitmapset * MHP_BTSET = NULL;

//
//bool plan(beginner_tutorials::AddTwoInts::Request  &req,
//         beginner_tutorials::AddTwoInts::Response &res )
//{
//
//}

bool planPath(human_nav_node::HANaviPlan::Request &req,
                            human_nav_node::HANaviPlan::Response &res) {
//  Pose start = req.start;
//  Pose goal = req.goal;
  return false;
}

bool initWorld(human_nav_node::InitWorld::Request &req,
                            human_nav_node::InitWorld::Response &res) {
//  Pose start = req.start;
//  Pose goal = req.goal;
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

//  NavigationNode *node = new NavigationNode();

  ros::ServiceServer planservice = n.advertiseService("HANaviPlan", planPath);
  ros::ServiceServer initservice = n.advertiseService("InitWorld", initWorld);
  ROS_INFO("Ready to plan.");
  ros::spin();

  return 0;
}
