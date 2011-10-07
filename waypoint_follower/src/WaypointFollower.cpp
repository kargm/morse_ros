/*
 * WaypointFollower.cpp
 * serves to calculate robot velocities in regular
 * intervals for a given path of waypoints.
 * If the intervals are too great, this will failto niecly follow a path, and oscillate instead.
 *
 *  Created on: May 13, 2009
 *      Author: kruset
 */

#include "WaypointFollower.h"
#include "M3DW_Utils.h"

#include <ros/ros.h>

#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <iostream>


/* the number of old waypoints to maintain on a stack */
#define MAX_DONE_WAYPOINTS 5


namespace NHPPlayerDriver {

  /**
   * pathSafetyDistance = how close another agent may be to path before the WP-follower stops to avoid collision
   */
  WaypointFollower::WaypointFollower(WP_FOLLOW_PARAMS params) {


    this->motionParams = params;

    position_initialized =  false; // whether robot is localized
    goal_ready = false; // whether waypoints have been sent
    robot_movement_allowed = false;

    humanOnPath = false;
    pathBlockedTimestamp = -1;

    waitUntil =0;
  }

  WaypointFollower::~WaypointFollower() {
  }

  void WaypointFollower::changeMotionParams(WP_FOLLOW_PARAMS params) {
    this->motionParams = params;
  }


  void WaypointFollower::updatePosition(float x, float y, float az)
  {
    this->position_initialized = true; // to avoid moving before initialisation
    this->x = x;
    this->y = y;
    this->az = az;
	ROS_DEBUG_NAMED("odom", "Waypoint Follower got odom pose %f, %f, %f", x, y, az);
  }

  /**
   * sets point x,y,az vales if position has been set.
   * returns false else
   */
  bool WaypointFollower::getPosition(XYTH_COORD *point)
  {
    if (this->position_initialized) {
        if (point != NULL) {
            point->x = this->x;
            point->y = this->y;
            point->th = this->az;
        }
        return true;
    }
    return false;
  }

  //void WaypointFollower::getNextWP(Waypoint *point)
  //{
  //  point->x = this->gx;
  //  point->y = this->gy;
  //  point->az = this->gaz;
  //}

  /**
   * sets point coordinates to goal if a goal has been set.
   * returns false else
   */
  bool WaypointFollower::getGoal(XYTH_COORD *point)
  {
    if (this->goal_ready == true) {
      if (this->waypoint_queue.empty()) {
        // return current position, as we turn in goal
        point->x = this->gx;
        point->y = this->gy;
        point->th = this->final_gaz;
      } else {
        point->x = this->waypoint_queue.back().x;
        point->y = this->waypoint_queue.back().y;
        point->th = this->final_gaz; // take the final goal angle, not the path final angle !!

      }
      return true;
    }
    return false;
  }

  /* sets goal to current position and stops calculating path on update method */
  void WaypointFollower::resetGoal()
  {
    this->gx = this->x;
    this->gy = this->y;
    this->gaz = this->final_gaz = this->az;
    this->robot_movement_allowed = false;
  }

  ///**
  // * returns true if current is exactly between last and next geometrically
  // */
  //bool isBetween(Waypoint *last, Waypoint *current, Waypoint *next) {
  //	double gaz1 = atan2(current->y - last->y, current->x - last->x);
  //	double gaz2 = atan2(next->y - current->y, next->x - current->x);
  //	return (abs(NORMALIZE(gaz2 - gaz1)) < 0.1);
  //}

  inline int WaypointFollower::prunePlan() {
	  //removes all waypoints of front of plan which have been passed

	  double loop_distance = -1;
	  int startIndex = 0;

	  if (!this->position_initialized) {
		  return 0;
	  }

	  std::list<XYTH_COORD> newWaypoints = this->getRemainingPath();
	  // the distance at which we can go towards next wp
	  double skipdistance;
	  if (newWaypoints.size() < 5) {
		  // stepwise reduce success distance when closing in on goal
		  skipdistance = this->motionParams.success_distance * this->waypoint_queue.size();
	  } else {
		  skipdistance = this->motionParams.wp_success_distance;
	  }

	  // assume first local minimum is the first point we care about (plausible global minimum)
	  std::list<XYTH_COORD>::iterator it = newWaypoints.begin();
	  XYTH_COORD cur_wp;
	  cur_wp.x = it->x;
	  cur_wp.y = it->y;
	  cur_wp.th = it->th;
	  double min_distance = DISTANCE2D(this->x, this->y, it->x, it->y);
	  it++;
	  for (; it != newWaypoints.end();
			  it++) {
		  loop_distance = DISTANCE2D(this->x, this->y, it->x, it->y);
		  ROS_DEBUG_NAMED("prune", "Loop distance: (%f, %f)-(%f,%f) =  %f",this->x, this->y, it->x, it->y, loop_distance);
			  if (loop_distance < min_distance) {
				  startIndex ++;
				  min_distance = loop_distance;
			  } else {
//				  ROS_DEBUG_NAMED("prune", "minimum found");
				  break;
			  }
	  }

	  int count = 0;
	  int skip = 0;
	  for (it = newWaypoints.begin(); it != newWaypoints.end();
			  it++) {
		  if (count < startIndex) {
			  skip++;
		  } else {
			  loop_distance = DISTANCE2D(this->x, this->y, it->x, it->y);
			  ROS_DEBUG_NAMED("prune", "Loop distance: (%f, %f)-(%f,%f) =  %f",this->x, this->y, it->x, it->y, loop_distance);
			  if (loop_distance < skipdistance) {
				  skip++;
			  } else {
				  break;
			  }
		  }
		  cur_wp.x = it->x;
		  cur_wp.y = it->y;
		  cur_wp.th = it->th;
	  }
	  // skip is index of first wp after minimum outside skipdistance
      // Variable it points one wp ahead of startDistance
	  // loopdistance is -1 or distance to wp at it

	  ROS_DEBUG_NAMED("prune", "Close point index: %d", skip);

	  // also if the distance robot - nextwaypoint is smaller than the distance both waypoints
	  if (loop_distance >= 0) {
		  double current_pace = DISTANCE2D(cur_wp.x, cur_wp.y, it->x, it->y);
		  ROS_DEBUG_NAMED("prune", "next %f, both: %f", loop_distance, current_pace);
		  if (loop_distance <= current_pace) {
			  skip++;
		  }
	  }

	  ROS_DEBUG_NAMED("prune", "Skipping points: %d", skip);
	  return skip;
  }

  int WaypointFollower::changeWaypoints(std::list<XYTH_COORD> &newWaypoints, float final_heading) {
    int result = OK;

    humanOnPath = false; // to be updated with new path later

    this->waypoint_queue.clear();
    for (std::list<XYTH_COORD>::iterator it = newWaypoints.begin();it != newWaypoints.end(); it++) {
            XYTH_COORD newPoint;
            newPoint.x = it->x;
            newPoint.y = it->y;
            newPoint.th = it->th;
            this->waypoint_queue.push_back(newPoint);
    }
    int skip = this->prunePlan();
    for (int var = 0; var < skip +1; ++var) {
        this->updateGoalForNextWaypoint();
	}
    this->final_gaz = final_heading;
    goal_ready = true;
    robot_movement_allowed = true;
    direction_already_computed_p = false;
    pathBlockedTimestamp = -1;

    ROS_DEBUG_NAMED("velo","Changed waypoints with final heading %f", this->final_gaz);
    return result;
  }

  std::list<XYTH_COORD> WaypointFollower::getRemainingPath() {
    return this->waypoint_queue;
//    for (it = this->waypoint_queue.begin();
//            it != this->waypoint_queue.end() && sumdist < 0.4 && i < 4; // 0.4 and 4 magic numbers, meaning 40 cm or 4 waypoints max
//            it++) {
  }


  /**************************************************************************
   * returns a velocity update command to reach the goal position
   * iterates over waypoints
   * ************************************************************************** */
  void WaypointFollower::updateVelocities(VELOCITY_COMMAND &cmd)
  {
    // set velocity mode

    cmd.state = 1;
    cmd.vel.px = 0;
    cmd.vel.py = 0;
    cmd.vel.pa = 0;

    if (this->goal_ready == false ||
        this->position_initialized == false ||
        this->robot_movement_allowed == false) {
      	ROS_ERROR("Robot blocked %d, %d, %d.", this->goal_ready, this->position_initialized, this->robot_movement_allowed);
      return;
    }

    timeval timeVal;
    if (gettimeofday(&timeVal, NULL) != 0) {
        ROS_ERROR("Fatal error when reading System time ");
        return;
    }

    // penalty for robot-robot when waiting for human-robot
    if (waitUntil > 0) { // check whether robot shall wait a little longer
      if (timeVal.tv_sec >= waitUntil) {
        waitUntil = -1; // wait time has passed
      } else {
        ROS_DEBUG("Waiting %ld more seconds...", waitUntil - timeVal.tv_sec);
        return;
      }
    }

    // TODO: if we think we stand still and position has not changed (by pushing), don't recalculate
    double current_distance_to_goal = DISTANCE2D(this->x,this->y,this->gx, this->gy);
    ROS_DEBUG_NAMED("velo", "Distance to goal: %f, waypoints: %d", current_distance_to_goal, this->waypoint_queue.size());

    if (this->humanOnPath == true) { // stop if human is in the way, but allow rotating on spot

      double turning_angle = NORMALIZE(this->gaz - this->az);
      ROS_DEBUG_NAMED("velo", "Turning angle: %f", turning_angle);
      pathBlockedTimestamp = timeVal.tv_sec;
      if (fabs(turning_angle) < this->motionParams.az_success_distance * 2) {
        cmd.vel.pa = 0;
      } else {
        cmd.vel.pa = ABSMAX( ( turning_angle * this->motionParams.pid_rot_kp / 3 ), this->motionParams.max_rot_vel );
      }
      return;

    } else {
      pathBlockedTimestamp = -1;
    }


    if (this->direction_already_computed_p == false) {
      // required to detect from call to call when we move accross a waypoint meanwhile
      this->last_known_distance_to_goal = current_distance_to_goal;
    }

    /* either we are at the end (within single success distance)
     * or we are on a waypoint (within 3x success distance)
     * or we need to continue and correct PID velocities
     */
    if (this->waypoint_queue.empty() && (current_distance_to_goal < this->motionParams.success_distance)) {
      ROS_DEBUG_NAMED("velo", "Success: (%f, %f) in delta %f of  (%f, %f)", this->gx, this->gy, current_distance_to_goal, this->x,  this->y);
      turnVelocityInGoal(cmd);
    } else {
    	// either more waypoints are left or we are not quite in final location

    	if (!this->waypoint_queue.empty() ) {
    		int skip = this->prunePlan();
    		for (int var = 0; var < skip; ++var) {
    			this->updateGoalForNextWaypoint();
    		}
            current_distance_to_goal = DISTANCE2D(this->x,this->y,this->gx, this->gy);
            this->last_known_distance_to_goal = current_distance_to_goal;
    	}

    	// for direction calculations
    	updateVelocityBetweenWaypoints(cmd, current_distance_to_goal);
    	this->last_known_distance_to_goal = current_distance_to_goal;

    }

  }



  bool WaypointFollower::hasActiveGoal() {
    return this->goal_ready;
  }

  bool WaypointFollower::isBlockedLongterm() {
    if (pathBlockedTimestamp > 0) {
      timeval timeVal;
      if (gettimeofday(&timeVal, NULL) != 0) {
        ROS_ERROR("Fatal error when reading System time. " );
        return false;
      }

      if (timeVal.tv_sec - pathBlockedTimestamp > 5) {
        ROS_ERROR("Robot blocked longterm by human.");
        return true;
      }
    }
    return false;
  }

  /** allows to tell robot to stop and wait for an amount of time at least before moving on **/
  void WaypointFollower::waitInPosition(long seconds) {
    long currentSecs = getCurrentSecs();
    if (currentSecs < 0) {
      ROS_ERROR("Fatal error: %ld ", currentSecs);
    } else {
      this->waitUntil = currentSecs + seconds;
    }
  }



  /**
   *
   */
  inline void WaypointFollower::turnVelocityInGoal(VELOCITY_COMMAND & cmd) {
    double final_turning_angle = 0.0;
    final_turning_angle = NORMALIZE(this->final_gaz - this->az);
    ROS_DEBUG_NAMED("velo","Normalize(%f - %f) = %f", this->final_gaz, this->az, final_turning_angle);
    ROS_DEBUG_NAMED("velo","Reaching final angle from %f to %f = %f", this->az, this->final_gaz, final_turning_angle);
    cmd.vel.px = 0;

    if (fabs(final_turning_angle) < this->motionParams.az_success_distance) {
      cmd.vel.pa = 0;
      ROS_DEBUG("Goal reached");
      this->goal_ready = false;
      this->robot_movement_allowed = false; // wait for new goal
    } else {
      cmd.vel.pa = ABSMAX( ( final_turning_angle * this->motionParams.pid_rot_kp ), this->motionParams.max_rot_vel );
    }
  }

  /**
   * selects the next waypoint of the trajectory
   */
  inline void WaypointFollower::updateGoalForNextWaypoint() {
	if (this->waypoint_queue.empty()) {
      return;
	}
	XYTH_COORD next_wp = this->waypoint_queue.front(); // take first item

    this->waypoint_queue.pop_front(); // remove first item from queue
    this->gx = next_wp.x;
    this->gy = next_wp.y;

    //humanOnPath = isHumanOnPath(agentOnPathSafetyDistance);
    humanOnPath = false; // checked by planner, safe max velocity is set.

    // the target angle on the next waypoint is the angle pointing to the waypoint after that, or the final angle
    if (this->waypoint_queue.empty()) {
      this->gaz = this->final_gaz;
    } else {
      XYTH_COORD over_next_wp = this->waypoint_queue.front();
      this->gaz = atan2(over_next_wp.y - next_wp.y, over_next_wp.x - next_wp.x);
    }
    ROS_DEBUG_NAMED("velo", "Going next to waypoint (%f, %f)", next_wp.x, next_wp.y);
  }

  /**
   * Calculates forward and turning velocities
   */
  inline void WaypointFollower::updateVelocityBetweenWaypoints(  VELOCITY_COMMAND & cmd, double current_distance_to_goal)
  {
    double turning_angle = 0.0;
    double local_trans_angle_range = 0.0, angle_towards_goal_absolute = 0.0;
    double start_driving_forward_turning_angle = 0.0, start_driving_backward_turning_angle = 0.0;

    // absolute angle the robot has to take to reach the goal if he moves forward
    angle_towards_goal_absolute = atan2(this->gy - this->y, this->gx - this->x);

    ROS_DEBUG_NAMED("velo", "Localplanner (%f, %f, %f) -> (%f, %f, %f) ", this->x, this->y, this->az, this->gx, this->gy, angle_towards_goal_absolute );

        // angular difference between current robot angle and angle_towards_goal_absolute if robot drives
    // forward - turning_angle_forward must be within [-PI, .. , PI]
    start_driving_forward_turning_angle = NORMALIZE(angle_towards_goal_absolute - this->az);
    // angular difference between current robot angle and angle_towards_goal_absolute if robot drives
    // backward - a clockwise forward-angle results in a counterclockwise backward-angle and vice versa
    start_driving_backward_turning_angle = NORMALIZE(start_driving_forward_turning_angle + M_PI);



    bool moved_away_from_target = false;
    // check if we are moving away from the target
    if (current_distance_to_goal > (this->last_known_distance_to_goal + this->motionParams.success_distance)) {
      moved_away_from_target = true; // if we moved beyond the goal already stop driving
    }

    // if we got a new waypoint, compute the new driving direction (forward or backward) to reach it
    if (!this->direction_already_computed_p || moved_away_from_target) {
      updateDrivingDirection(angle_towards_goal_absolute, current_distance_to_goal, start_driving_forward_turning_angle, start_driving_backward_turning_angle);
      this->direction_already_computed_p = true;
    } // endif ! direction_already_computed

    // set turning_angle according to movement direction
    turning_angle = ( this->driving_direction == DRIVING_DIRECTION_FORWARD
        ? start_driving_forward_turning_angle
            : start_driving_backward_turning_angle );

    // whether the robot will have to change directions in the next WP
    XYTH_COORD over_next_wp = this->waypoint_queue.front();
    double angle_towards_over_next_wp = fabs(atan2(over_next_wp.y  - this->y, over_next_wp.x  - this->x));
    bool pathchanges = NORMALIZE(angle_towards_over_next_wp - this->az) > 0.7;

    // local_trans_angle_range is the angle outside which the robot should just turn and not yet move forward
    // allow moving forward within a range bounded by 0.1 and maximally 0.75 (45 degrees) in general,
    // or max according to params and distance to next goal (less tolerant near goal)
    if (this->waypoint_queue.empty()) {
        local_trans_angle_range = 0.1;
    } else {
      // value between 0.3 and 1.2
        local_trans_angle_range = MINMAX(this->motionParams.trans_angle_range * fmin(current_distance_to_goal * 3, 1.0), 0.4, 1.2);
    }

    if (fabs(turning_angle) > local_trans_angle_range )
      { // rotation without driving, because turning_angle is large in comparison to current_distance to goal, or we are at goal
        cmd.vel.px = 0.00;
        // set rotation speed to a value that is proportional to turning_angle, but below a certain cut-off value (max_rot_vel)
        cmd.vel.pa = ABSMAX( ( turning_angle * this->motionParams.pid_rot_kp), this->motionParams.max_rot_vel );
        ROS_DEBUG_NAMED("velo", "Turning because angle %f > %f : %f", fabs(turning_angle), local_trans_angle_range, cmd.vel.pa);
      } else { // rotating and driving simultaneously, because turning_angle is not large in comparison to current_distance to goal
          // set translation speed proportional to current_distance to the goal
          // TODO (maybe): if goal angle is similar to current angle (no hard edge ahead) and next waypoint is not the last, then don't break down as much.
          if (humanOnPath) {
            cmd.vel.px = 0;
          } else if (this->waypoint_queue.empty()) {
            // last waypoint, slow down. px is in dm/s, not m/s!!
            cmd.vel.px = this->driving_direction *
                fmin(this->motionParams.max_trans_vel / 10 /*dm/s*/, ( current_distance_to_goal * this->motionParams.pid_trans_kp ));
          } else if (this->waypoint_queue.size() < 3) {
            // last few waypoints, slow down. px is in dm/s, not m/s!!
            cmd.vel.px = this->driving_direction *
                fmin(this->motionParams.max_trans_vel / 10 /*dm/s*/, ( current_distance_to_goal *2 * this->motionParams.pid_trans_kp ));
          } else if (pathchanges) {
            cmd.vel.px = this->driving_direction *
                                fmin(this->motionParams.max_trans_vel / 10 /*dm/s*/, ( fmax(1, (this->waypoint_queue.size() * 0.1)) * this->motionParams.pid_trans_kp  ));
          } else { // more speed in middle waypoints
              cmd.vel.px = this->driving_direction *
                  fmin(this->motionParams.max_trans_vel / 10 /*dm/s*/, ( fmax(1, (this->waypoint_queue.size() * 0.2)) * this->motionParams.pid_trans_kp  ));
          }
          // set rotation speed to a value that is proportional, but below a certain cut-off value (max_rot_vel)
          cmd.vel.pa = ABSMAX( ( turning_angle * this->motionParams.pid_rot_kp ), this->motionParams.wp_max_rot_vel  );
      }


  } // end updateVelocityBetweenWaypoints


  /**
   * returns forward or backward
   */
  inline void WaypointFollower::updateDrivingDirection(
      double angle_towards_goal_absolute,
      double & current_distance_to_goal,
      double & start_driving_forward_turning_angle,
      double & start_driving_backward_turning_angle)
  {
    double goal_driving_forward_turning_angle  = NORMALIZE(this->gaz - angle_towards_goal_absolute);
    double goal_driving_backward_turning_angle = NORMALIZE(this->gaz - (angle_towards_goal_absolute - M_PI));

    //				PLAYER_MSG4( 2, "sdfta: %f / gdfta: %f / sdbta: %f / gdbta: %f",
    //						start_driving_forward_turning_angle, goal_driving_forward_turning_angle,
    //						start_driving_backward_turning_angle, goal_driving_backward_turning_angle );
    //				PLAYER_MSG2( 2, "Sum driving_forward_angles: %f / Sum driving_backward_angles: %f\n",
    //						(fabs(start_driving_forward_turning_angle) + fabs(goal_driving_forward_turning_angle)),
    //						(fabs(start_driving_backward_turning_angle) + fabs(goal_driving_backward_turning_angle)) );

    // if we are > 1m away from the goal or we have to rotate less than 90deg. to face the goal -> drive forward
    // -0.1 is a magic number that prefers driving forward even if the forward angles are bigger
    if ( current_distance_to_goal > 1.0 ||
        ((fabs(start_driving_forward_turning_angle) + fabs(goal_driving_forward_turning_angle) - 0.1) <=
            (fabs(start_driving_backward_turning_angle) + fabs(goal_driving_backward_turning_angle))) ) {
      this->driving_direction = DRIVING_DIRECTION_FORWARD;
    }
    else {
      this->driving_direction = DRIVING_DIRECTION_BACKWARD;
    }
  }

}
