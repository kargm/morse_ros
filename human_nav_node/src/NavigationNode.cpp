/*
 * NavigationNode.cpp
 *
 *  Created on: Mar 19, 2011
 *      Author: thibault
 */

#include "ros/ros.h"


#include <iostream>
#include <boost/program_options.hpp>
#include <string>


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

using namespace std;
namespace human_nav_node {


MotionPlanner planner;

void fillHuman(MHP_HUMAN_POSITION &pos, std::vector<human_nav_node::HumanState> humans, unsigned int id) {
	if (id < humans.size()) {
		human_nav_node::HumanState state = humans[id];
		pos.id = id;
		pos.pos.x = state.simpleBodyPose.position.x;
		pos.pos.y = state.simpleBodyPose.position.y;
		pos.pos.th = state.simpleBodyPose.orientation.z;

		if (state.locked) {
			if (state.moving) {
				pos.state = MHP_MOVING;
			} else {
				pos.state = MHP_STANDING_TRANSPARENT;
			}
		} else {
			pos.state = MHP_STANDING;
		}
	} else {
		pos.id = -1; // does not exist
	}
}

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
	fillHuman(requestStruct.humpos1, humans, 0);
	fillHuman(requestStruct.humpos2, humans, 1);
	fillHuman(requestStruct.humpos3, humans, 2);
	fillHuman(requestStruct.humpos4, humans, 3);
	fillHuman(requestStruct.humpos5, humans, 4);
	// further humans are ignored, limitation of MHP. TODO, use MHP parameter.

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

namespace po = boost::program_options;

int main(int argc, char **argv)
{
	bool showViz = true;

	po::options_description desc("Allowed options");
	desc.add_options()
	    		("help,h", "produce help message")
	    		("p3dfile,f", po::value<string>(), "in P3D format (*.p3d)")
	    		("scenario,s", po::value<string>(), "in P3D scenario format (*.sce)")
	    		("novisualization,n", "do not show 3d visualization")
	    		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 0;
	}

	if (vm.count("p3dfile")) {
		cout << "Using world file " << vm["p3dfile"].as<string>() << ".\n";
		human_nav_node::planner.init( vm["p3dfile"].as<string>().c_str(), showViz);
	} else {
		cout << "World file level was not set.\n";
	}

	if (vm.count("scenario")) {
		cout << "Using scenario file " << vm["scenario"].as<string>() << ".\n";
		human_nav_node::planner.initScene(vm["scenario"].as<string>().c_str());

	} else {
		cout << "Scenario file level was not set.\n";
	}

	if (vm.count("novisualization")) {
		cout << "Not showing visualization.\n";
		showViz = false;
	}

	ros::init(argc, argv, "hanp_plan_server");
	ros::NodeHandle n;

	ros::ServiceServer planService = n.advertiseService("HANaviPlan", human_nav_node::planPath);
	ros::ServiceServer initService = n.advertiseService("InitWorld", human_nav_node::initWorld);
	ros::ServiceServer initSceneService = n.advertiseService("InitScenario", human_nav_node::initScenario);
	ros::ServiceServer setParamsService = n.advertiseService("ChangeInterfaceParams", human_nav_node::changeInterfaceParams);
	ros::ServiceServer setCamService = n.advertiseService("ChangeCamPos", human_nav_node::changeCamPos);

	ROS_INFO("services started.");

	ros::Rate loop_rate(5);

	while(ros::ok()) {
		ros::spinOnce();
		human_nav_node::spinInterface();
		loop_rate.sleep();
	}

	return 0;
}


