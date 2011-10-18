#ifndef NHPDRIVER_H_
#define NHPDRIVER_H_


#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <waypoint_follower/WaypointFollowerConfig.h>

#include <M3DW_Utils.h>
#include "WPFollowerParams.h"
#include "WaypointFollower.h"
#include "HumanTracker.h"
#include "PositionProjection.h"

// for some experiments, human should just go on at desired speed
#define HUMAN_BREAKS_FOR_ROBOT FALSE

// forward declarations
namespace NHPPlayerDriver
{
  class WaypointFollower;
  class HumanTracker;
  class PoseHistory;
}

struct AbsoluteTrajectory;

namespace human_wp_follow_plugin{


// Driver for go to pose as pid controller
class WaypointFollowerRos : public nav_core::BaseLocalPlanner
{

public:
  // Constructor
  WaypointFollowerRos() : costmap_ros_(NULL), tf_(NULL), initialized_(false){

  }

  WaypointFollowerRos(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false){
	initialize(name, tf, costmap_ros);
  }

  ~WaypointFollowerRos();

  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

//  //update velocities of robot
//  void UpdateVelocities(NHPPlayerDriver::VELOCITY_COMMAND &cmd);

  void perceptHuman(NHPPlayerDriver::XYTH_COORD &dat_pos, int i);

  void perceptRobotPose(float px, float py, float pitch);

private:

  /**
   * @brief  Callback for receiving odometry data
   * @param msg An Odometry message
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void reconfigure_callback(waypoint_follower::WaypointFollowerConfig &config, uint32_t level);

  /**
   * Called whenever there is a goto command, or when the environment (the position of humans) changed.
   * safe flag specifies robot will not plan path that collides with human, unsafe means we try to make human move
   */
  void adaptMotionParams();


  std::ofstream logfile;

  NHPPlayerDriver::WaypointFollower *waypoint_follower;
  NHPPlayerDriver::HumanTracker *human_tracker;
  NHPPlayerDriver::PoseHistory *robTrack;

  dynamic_reconfigure::Server<waypoint_follower::WaypointFollowerConfig> *reconfigure_server;


  std::string mboxname;
  std::string p3d_filepath;
  bool last_player_command_was_stop;
  // robot in simulation acting as human
  bool isHuman;

  costmap_2d::Costmap2DROS* costmap_ros_;
  tf::TransformListener* tf_;
  bool initialized_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  ros::Subscriber odom_sub_;

  int humansAwareOf;
};

}

#endif /* NHPDRIVER_H_ */
