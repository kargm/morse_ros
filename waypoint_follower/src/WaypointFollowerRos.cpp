#include "WaypointFollowerRos.h"

#include <pluginlib/class_list_macros.h>

#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>



#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <nav_core/base_local_planner.h>


// Cannot load library: No manifest in /home/kruset/work/ros/other_stacks/morse_ros/waypoint_follower/lib/libwp_follower.so: human_wp_follow_plugin__WaypointFollowerRos
//human_wp_follow_plugin__HumanAwareWaypointFollower
//human_wp_follow_plugin__WaypointFollowerRos
//register this planner as a BaseWaypointFollowerRos plugin
PLUGINLIB_DECLARE_CLASS(waypoint_follower, WaypointFollowerPlugin, human_wp_follow_plugin::WaypointFollowerRos, nav_core::BaseLocalPlanner)


namespace human_wp_follow_plugin{


//by how much of the original max speed we reduce the max speed in each cycle to find a non-colliding suitable speed
#define SPEED_ADAPTION_STEPS 0.28


#define LOGGING FALSE


/* ************************************************************************** */
/* Driver registration and init                                               */
/* ************************************************************************** */

using namespace NHPPlayerDriver;

double PROJECTION_TIMESPAN = 6.0;

// for center to center distance between human and robot lower than this (in meters), consider them in collision
double COLLISION_DISTANCE = 1.0;

int flushCountdown= 20;
XYTH_COORD logPos;


void WaypointFollowerRos::initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
	if(!initialized_){
	  costmap_ros_ = costmap_ros;
      tf_ = tf;

      robTrack = new PoseHistory();
      human_tracker = new HumanTracker();
      WP_FOLLOW_PARAMS params = getRobotMotionParams();
      this->waypoint_follower = new WaypointFollower(params);



	  last_player_command_was_stop = false;
	  initialized_ = true;

	  // subscribe to Odom topic for velocity
//	  ros::NodeHandle gn;
//	  odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom",
//			  1,
//			  boost::bind(&WaypointFollowerRos::odomCallback, this, _1));
	  ROS_DEBUG("Waypoint Follower initialized");
	}
}


WaypointFollowerRos::~WaypointFollowerRos()
{
  delete this->waypoint_follower;
  delete this->human_tracker;
  delete this->robTrack;
}


bool WaypointFollowerRos::isGoalReached() {
	return ! waypoint_follower->hasActiveGoal();
}

bool WaypointFollowerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	ROS_DEBUG_NAMED("plan", "Waypoint Follower got new plan");

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    std::vector<geometry_msgs::PoseStamped> transformed_plan = global_plan_;
//    if(! base_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan)){
//    	ROS_WARN("Could not transform the global plan to the frame of the controller");
//    	return false;
//    }
    if(transformed_plan.empty()) {
    	ROS_WARN("Waypoint Follower got empty plan");
    	return true;
    }

    float finalheading = 0.0;
    std::list<XYTH_COORD> newWaypoints;

    for( std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_plan.begin(); it != transformed_plan.end(); ++it) {
        XYTH_COORD wp;
        geometry_msgs::PoseStamped planwp = *it;
        wp.x = planwp.pose.position.x;
        wp.y = planwp.pose.position.y;
        newWaypoints.push_back(wp);
        btScalar roll,pitch,yaw;
        tf::Stamped<tf::Pose> tfpose;
        tf::poseStampedMsgToTF(planwp, tfpose);
        btMatrix3x3(tfpose.getRotation()).getRPY(roll, pitch, yaw, 1);
        finalheading = yaw;
        ROS_DEBUG_NAMED("plan", "Waypoint Follower waypoint: (%f, %f, %f)", planwp.pose.position.x, planwp.pose.position.y, yaw);
    }


    int wps_ok = waypoint_follower->changeWaypoints(newWaypoints, finalheading);
    if (wps_ok != OK) {
    	ROS_WARN("Waypoint Follower got invalid plan");
    	//memory fault, perhaps because of multi-threading
    	this->waypoint_follower->resetGoal();
    	return false;
    }

	return true;
}




bool WaypointFollowerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	if(! initialized_){
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	VELOCITY_COMMAND drive_cmds;

	tf::Stamped<tf::Pose> global_pose;
	if(!costmap_ros_->getRobotPose(global_pose)) {
		ROS_ERROR("Failed to get robot position");
		return false;
	}


	btScalar roll,pitch,yaw;
	btMatrix3x3(global_pose.getRotation()).getRPY(roll, pitch, yaw, 1);
	ROS_DEBUG_NAMED("cyclic","Robot position: %f, %f, %s ", global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), global_pose.frame_id_.c_str());

	this->robTrack->addPoseIsValid(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), roll,pitch,yaw);

	ROS_DEBUG_NAMED("cyclic","Robot pose: %f, %f, %s ", global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), global_pose.frame_id_.c_str());

	this->waypoint_follower->updatePosition(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw);

	adaptMotionParams();

	waypoint_follower->updateVelocities(drive_cmds);

	ROS_DEBUG_NAMED("cyclic", "Waypoint Follower created velocities x = %f, th=%f", drive_cmds.vel.px, drive_cmds.vel.pa);

	//pass along drive commands
	cmd_vel.linear.x = drive_cmds.vel.px;
	cmd_vel.linear.y = drive_cmds.vel.py;

	cmd_vel.angular.z = drive_cmds.vel.pa;

	return true;
}


void WaypointFollowerRos::perceptHuman(NHPPlayerDriver::XYTH_COORD &dat_pos, int id)
{
	int result;
	if (id < 0 && id > 5) { // should never happen
	      ROS_ERROR("Error,bad ID %d", id);
	      result = 0;
	  }

	  if (human_tracker->handlePositionUpdate(dat_pos.x, dat_pos.y, dat_pos.th, id)) {
		  result = 1;
	  }
	  result = 0;
}

//void WaypointFollowerRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
//   //we assume that the odometry is published in the frame of the base
//   boost::mutex::scoped_lock(odom_mutex_);
//   ROS_DEBUG_NAMED("odom", "Waypoint Follower got odom pose %f, %f, %f", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
//   WaypointFollowerRos::perceptRobotPose(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
// }

/**
   * Adapts params according to prediction to avoid conflicts
   */
  void WaypointFollowerRos::adaptMotionParams()
  {
    std::list<XYTH_COORD> waypoints = this->waypoint_follower->getRemainingPath();
    XYTH_COORD humanPoses[this->human_tracker->getHumanMaxNo()];
    VELOCITY humanVelocities[this->human_tracker->getHumanMaxNo()];
    double reduceAmount = 0;
    int num_humans = 0;
    for (int i = 0; i < this->human_tracker->getHumanMaxNo(); ++i) {
        Human* human = this->human_tracker->getHuman(i);
        if (human->exists) {
            humanPoses[i].x = human->x;
            humanPoses[i].y = human->y;
            humanPoses[i].th = human->az;
            num_humans++;
            humanVelocities[i] = this->human_tracker->getHumanVelocity(i);
        }
    }

    WP_FOLLOW_PARAMS params = getRobotMotionParams();
    double original_max_trans_vel = params.max_trans_vel;
    bool safeMotionPossible = false;
    // test several different motion param variants until we find one that does not conflict (hopefully the first)
    for (reduceAmount = 0; reduceAmount < 1; reduceAmount += SPEED_ADAPTION_STEPS) {
        //      printf("%f\n", reduceAmount);
        params.max_trans_vel = (1 - reduceAmount) * original_max_trans_vel;
        XYTH_COORD currentpos;
        this->robTrack->valid2DPose(&currentpos);
        if (! checkPosesInConflict(humanPoses,
            humanVelocities,
            num_humans,
            currentpos,
            this->robTrack->velocity(),
            &waypoints,
            PROJECTION_TIMESPAN,
            COLLISION_DISTANCE,
            params)) {
            safeMotionPossible = true;
            break;
        } else {
            //          ROS_DEBUG( "Reducing max speed proportionally to %f\n", (1 - reduceAmount) * original_max_trans_vel);
        }
    }
    if (!safeMotionPossible) {
        params.max_trans_vel = 0;
    }
    // even if loop found no solution, max velocity is a low setting
    waypoint_follower->changeMotionParams(params);

  }

}


