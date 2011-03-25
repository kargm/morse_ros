#ifndef HUMAN_AWARE_NAVIGATION_H
#define HUMAN_AWARE_NAVIGATION_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "human_nav_node/InitWorld.h"
#include "human_nav_node/InitScenario.h"
#include "human_nav_node/HANaviPlan.h"
#include "human_nav_node/ChangeInterfaceParams.h"
#include "human_nav_node/Move3dInterfaceParams.h"
#include "human_nav_node/ChangeCamPos.h"
//#include "human_nav_node/MotionPlanner.h"
#include "human_nav_node/HumanState.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <human_aware_navigation/Move3dXmlRpcClient.hpp>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
//#include <costmap_2d/costmap_2d_ros.h>

//global representation
#include <nav_core/base_global_planner.h>

namespace human_nav_plugin{

class HumanAwareNavigation: public nav_core::BaseGlobalPlanner{
public:

  /**
   * @brief  Default constructor for the NavFnROS object
   */
  HumanAwareNavigation(){};

  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  //HumanAwareNavigation(std::string name, costmap_2d::Costmap2DROS* costmap_ros){}


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string, costmap_2d::Costmap2DROS*) {};

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~HumanAwareNavigation(){};

private:

}; // end class
}; // end namespace

void humanPoseCallback(const geometry_msgs::PoseStamped& pose);

void initSub();
void initPub();

// Interface to planner services
bool initWorld(std::string& filename, bool graphics);
bool initScene(std::string& filename);
nav_msgs::Path planPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, const human_nav_node::HumanState* humanPosesArray, const int& numberOfHumanPoses);
void changeIFace(bool showObst, bool showDist, bool showVis, bool showShadows);
void changeCamPos(float xdest, float ydest, float zdest, float dist, float hrot, float vrot);

#endif
