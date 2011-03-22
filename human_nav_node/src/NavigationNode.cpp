/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include "ros/ros.h"



#include "human_nav_node/InitWorld.h"
#include "human_nav_node/InitScenario.h"
#include "human_nav_node/HANaviPlan.h"
#include "human_nav_node/ChangeInterfaceParams.h"
#include "human_nav_node/Move3dInterfaceParams.h"
#include "human_nav_node/ChangeCamPos.h"
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
	resultTraj.no = 0;
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
                            InitWorld::Response &res)
{
  int result = planner.init(req.pdfilename.c_str(), req.showInterface == 1);
  res.resultcode = result;
  if (result == 0) {
      return true;
  } else {
      return false;
  }
}

bool initScenario(InitScenario::Request &req,
		InitScenario::Response &res)
{
  int result = planner.initScene(req.scfilename.c_str());
  res.resultcode = result;
  if (result == 0) {
      return true;
  } else {
      return false;
  }
}


bool changeInterfaceParams(ChangeInterfaceParams::Request &req,
		ChangeInterfaceParams::Response &res)
{
	MHP_INTERFACE_PARAMS interfaceParam;
	interfaceParam.height = req.params.height;
	interfaceParam.width = req.params.width;
	interfaceParam.walls = req.params.walls ? GEN_TRUE :GEN_FALSE;
	interfaceParam.floor = req.params.floor ? GEN_TRUE :GEN_FALSE;
	interfaceParam.tiles = req.params.tiles ? GEN_TRUE :GEN_FALSE;
	interfaceParam.shadows = req.params.shadows ? GEN_TRUE :GEN_FALSE;
	interfaceParam.saveInterface = req.params.saveInterface ? GEN_TRUE :GEN_FALSE;
	interfaceParam.show_nav_obstacles = req.params.show_nav_obstacles ? GEN_TRUE :GEN_FALSE;
	interfaceParam.show_nav_distance_grid = req.params.show_nav_distance_grid ? GEN_TRUE :GEN_FALSE;
	interfaceParam.show_nav_visibility_grid = req.params.show_nav_visibility_grid ? GEN_TRUE :GEN_FALSE;
	interfaceParam.show_nav_hidzones_grid = req.params.show_nav_hidzones_grid ? GEN_TRUE :GEN_FALSE;


	int report;
	int result = planner.setInterfaceParams(&interfaceParam, &report);

	if (result == 0) {
		return true;
	} else {
		return false;
	}
}

bool changeCamPos(ChangeCamPos::Request &req,
		ChangeCamPos::Response &res)
{
	MHP_CAM_POS cam_pos;
	cam_pos.dist = req.dist;
	cam_pos.xdest = req.xdest;
	cam_pos.ydest = req.ydest;
	cam_pos.zdest = req.zdest;
	cam_pos.hrot = req.hrot;
	cam_pos.vrot = req.vrot;

	int report = 0;
	planner.changeCameraPosMain(&cam_pos, &report);
	return true;
}



void spinInterface() {
	planner.updateInterface();
}



}// end namespace



//************* MAIN *****************

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanp_plan_server");
  ros::NodeHandle n;

//  NavigationNode *node = new NavigationNode();

  ros::ServiceServer planService = n.advertiseService("HANaviPlan", human_nav_node::planPath);
  ros::ServiceServer initService = n.advertiseService("InitWorld", human_nav_node::initWorld);
  ros::ServiceServer initSceneService = n.advertiseService("InitScenario", human_nav_node::initScenario);
  ros::ServiceServer setParamsService = n.advertiseService("ChangeInterfaceParams", human_nav_node::changeInterfaceParams);
  ros::ServiceServer setCamService = n.advertiseService("ChangeCamPos", human_nav_node::changeCamPos);
  ROS_INFO("Ready to plan.");

  ros::Rate loop_rate(5);

  while(ros::ok()) {
	  ros::spinOnce();
	  human_nav_node::spinInterface();
	  loop_rate.sleep();
  }

  return 0;
}


