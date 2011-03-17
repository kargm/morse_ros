#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <sstream>
#include <pluginlib/class_list_macros.h>
#include <human_aware_navigation/human_aware_navigation.h>

PLUGINLIB_REGISTER_CLASS(HumanAwareNavigation, human_aware_navigation::HumanAwareNavigation, nav_core::BaseGlobalPlanner);
/**
 * A human-aware-navigation-plugin to replace the global planner of nav_core
 */

namespace human_aware_navigation{

int HumanAwareNavigation() {
  // constructor
}

bool HumanAwareNavigation::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan){

  // create dummy path consisting of 2 poses for testing (will be removed later)
  ROS_INFO("[sbpl_lattice_planner] getting start point (%g,%g) goal point (%g,%g)",
               start.pose.position.x, start.pose.position.y,goal.pose.position.x, goal.pose.position.y);

  geometry_msgs::PoseStamped pose1;
  pose1.header.stamp = ros::Time::now();
  pose1.header.frame_id = "odom";
  pose1.pose.position.x = 1;
  pose1.pose.position.y = 1;
  pose1.pose.orientation.w = 1;

  geometry_msgs::PoseStamped pose2;
  pose2.header.stamp = ros::Time::now();
  pose2.header.frame_id = "odom";
  pose2.pose.position.x = 2;
  pose2.pose.position.y = 1;
  pose2.pose.orientation.w = 1;

  plan.push_back(pose1);
  plan.push_back(pose2);

  // read PoseStamped array from xmlrpc and fill it into plan-vector

  // optional: Post path message for visualization

  return true;
}
};
