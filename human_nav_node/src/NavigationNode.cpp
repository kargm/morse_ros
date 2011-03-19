/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include "ros/ros.h"



//
//bool plan(beginner_tutorials::AddTwoInts::Request  &req,
//         beginner_tutorials::AddTwoInts::Response &res )
//{
//
//}

bool plan() {
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

//  NavigationNode *node = new NavigationNode();

//  ros::ServiceServer service = n.advertiseService("hanp_plan", plan);
  ROS_INFO("Ready to plan.");
  ros::spin();

  return 0;
}
