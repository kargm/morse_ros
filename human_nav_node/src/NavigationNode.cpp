/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include <human_nav_node/NavigationNode.h>

NavigationNode::NavigationNode() {}

NavigationNode::~NavigationNode() {}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("hanp_plan", add);
  ROS_INFO("Ready to plan.");
  ros::spin();

  return 0;
}
