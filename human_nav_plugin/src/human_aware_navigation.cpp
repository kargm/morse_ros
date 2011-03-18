#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <sstream>
#include <pluginlib/class_list_macros.h>

#include <human_aware_navigation/human_aware_navigation.h>
#include <human_aware_navigation/Move3dXmlRpcClient.hpp>

PLUGINLIB_DECLARE_CLASS(human_nav_plugin, HumanAwareNavigation, human_nav_plugin::HumanAwareNavigation, nav_core::BaseGlobalPlanner)
/**
 * A human-aware-navigation-plugin to replace the global planner of nav_core
 */

namespace human_nav_plugin{

bool HumanAwareNavigation::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){

  // create dummy path consisting of 2 poses for testing (will be removed later)
  ROS_INFO("[human_navigation] getting start point (%g,%g) goal point (%g,%g)",
               start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);
  plan.clear();


  /*geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = ros::Time::now();
  pose1.header.frame_id = "map";
  pose1.pose.position.x = -2.39708237648;
  pose1.pose.position.y = -0.0767308101058;
  pose1.pose.position.z = -0.0363690070808;
  pose1.pose.orientation.z = 0.0124436113983;
  pose1.pose.orientation.w = 0.999922573566;

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = ros::Time::now();
  pose2.header.frame_id = "map";
  pose2.pose.position.x = -2.99708237648;
  pose2.pose.position.y = -0.0767308101058;
  pose2.pose.position.z = -0.0363690070808;
  pose2.pose.orientation.z = 0.0124436113983;
  pose2.pose.orientation.w = 0.999922573566;

  plan.push_back(pose1);
  plan.push_back(pose2);
  plan.push_back(pose1);
  plan.push_back(pose2);
  plan.push_back(pose1);*/

  int port = 7011; // 7011 standard port
  const char* host = "lapbeetz6";

  Moved3dXmlRpcClient* c =  new Moved3dXmlRpcClient(host, port);

  AbsoluteTrajectory waypoints; // result
  Human humans[MHPD_MAX_HUMANS];

  Human h1;
  h1.x = -1.5;
  h1.y = -1.5;
  h1.az = 0.0;
  h1.pose = STANDING_TRANSPARENT;
  h1.locked = 0;
  h1.exists = 1;
  h1.lastMoveTimeStampSecs = 0;
  humans[0] = h1;

  int human_no = 1;

  c->planPath(-2.0, 0.0, 0.0, // pos
                      -1.0, 0.0, 0.0, // goal
                      humans, human_no, // humans
                      &waypoints,
                      0);

  std::cout << "Got path with costs " << waypoints.costs;

  for(int i=0; i<waypoints.numberOfSegments; i++) {
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.stamp = ros::Time::now();
    pose_tmp.header.frame_id = "map";
    pose_tmp.pose.position.x = waypoints.seg[i].x;
    pose_tmp.pose.position.y = waypoints.seg[i].y;
    ROS_INFO("[human_navigation] Added waypoint: (%g,%g)",
                   pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    plan.push_back(pose_tmp);
  }

  //ROS_INFO("[human_navigation] first waypoint is: (%g,%g)",
  //               pose1.pose.position.x, pose1.pose.position.y);

  ROS_INFO("[human_navigation] plan has size %d", plan.size());

  // read PoseStamped array from xmlrpc and fill it into plan-vector

  // optional: Post path message for visualization

  return true;
}
};
