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
#include "human_nav_node/HumanState.h"

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

namespace human_nav_node {

using namespace std;

MotionPlanner planner;

bool planPath(HANaviPlan::Request &req,
		HANaviPlan::Response &res) {
	geometry_msgs::Pose start = req.request.start;
	geometry_msgs::Pose goal = req.request.goal;
	std::vector<human_nav_node::HumanState> humans = req.request.humans;

	MHP_UPD_FINDPATH requestStruct;
	requestStruct.search_definition.startpos.x = start.position.x;
	requestStruct.search_definition.startpos.y = start.position.y;
	requestStruct.search_definition.startpos.th = start.orientation.z;
	requestStruct.search_definition.goalpos.x = goal.position.x;
	requestStruct.search_definition.goalpos.y = goal.position.y;
	requestStruct.search_definition.goalpos.th = goal.orientation.z;

	requestStruct.search_definition.configuration = 0;
	requestStruct.search_definition.linelen = 0;

	// TODO: map id to numbers
	requestStruct.humpos1.id = -1;
	requestStruct.humpos2.id = -1;
	requestStruct.humpos3.id = -1;
	requestStruct.humpos4.id = -1;
	requestStruct.humpos5.id = -1;

	cout<<"Requesting path ...\n";
	int result;
	MHP_NAV_TRAJECTORY resultTraj;
	planner.updPosAndFindNavTrajExec(requestStruct, resultTraj, &result);
	cout<<"Result: "<<result<<"\n";

	nav_msgs::Path path;
	for (int i = 0; i < resultTraj.no; ++i) {
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = resultTraj.xcoord[i];
		pose.pose.position.y = resultTraj.ycoord[i];
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = resultTraj.theta[i];
		path.poses.push_back(pose);
	}
	res.path = path;
	res.cost = resultTraj.cost;
	return true;
}

bool initWorld(InitWorld::Request &req,
                            InitWorld::Response &res) {
  int result = planner.init(req.pdfilename.c_str(), req.showInterface == 1);
  res.resultcode = result;
  if (result == 0) {
      return true;
  } else {
      return false;
  }
}

void spinInterface() {
	planner.updateInterface();
}
}// end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

//  NavigationNode *node = new NavigationNode();

  ros::ServiceServer planservice = n.advertiseService("HANaviPlan", human_nav_node::planPath);
  ros::ServiceServer initservice = n.advertiseService("InitWorld", human_nav_node::initWorld);
//  ros::ServiceServer initservice = n.advertiseService("StartInterfaceThread", human_nav_node::startUpdateLoop);
//  ros::ServiceServer initservice = n.advertiseService("StopInterfaceThread", human_nav_node::stopUpdateLoop);
  ROS_INFO("Ready to plan.");

  ros::Rate loop_rate(5);

  while(ros::ok()) {
	  ros::spinOnce();
	  human_nav_node::spinInterface();
	  loop_rate.sleep();
  }

  return 0;
}


