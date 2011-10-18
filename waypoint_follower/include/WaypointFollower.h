

/*
 * WaypointFollower.h
 *
 *  Created on: May 13, 2009
 *      Author: kruset
 */

#ifndef WAYPOINTFOLLOWER_H_
#define WAYPOINTFOLLOWER_H_

#include <list>
#include "WPFollowerParams.h"
#include <M3DW_Utils.h>


namespace NHPPlayerDriver
{

class WaypointFollower {

public: enum driving_direction_t {
	DRIVING_DIRECTION_FORWARD = 1,
	DRIVING_DIRECTION_BACKWARD = -1
};

public:
  WaypointFollower();
  virtual ~WaypointFollower();
  void updateVelocities(VELOCITY_COMMAND &cmd);
  void updatePosition(float x, float y, float az);
  bool getPosition(XYTH_COORD *point);
  bool getGoal(XYTH_COORD *point);
  void resetGoal();
  bool hasActiveGoal();
  bool isBlockedLongterm();
  int changeWaypoints(std::list<XYTH_COORD> &newWaypoints, float final_heading/*, double max_vel, float pid_rot_kp, double max_rot_vel*/);
  std::list<XYTH_COORD> getRemainingPath();
  void changeMotionParams(WP_FOLLOW_PARAMS params);
  WP_FOLLOW_PARAMS getMotionParams();
  void waitInPosition(long seconds);

WP_FOLLOW_PARAMS motionParams;

private:
  //bool isHumanOnPath(float safetyDistance);

  void turnVelocityInGoal(VELOCITY_COMMAND & cmd);
  void updateGoalForNextWaypoint();
  void updateVelocityBetweenWaypoints(VELOCITY_COMMAND & cmd, double current_distance_to_goal);
  void updateDrivingDirection(double angle_towards_goal_absolute, double & current_distance_to_goal, double & start_driving_forward_turning_angle, double & start_driving_backward_turning_angle);
  int prunePlan();
  //private:	void changeGoalPosition(float x, float y, float az, double max_vel, float pid_rot_kp, double max_rot_vel );

  //  // the last percept
  float x, y, az;
  // the goal position
  float gx, gy, gaz, final_gaz;

  bool position_initialized;
  bool goal_ready;
  bool robot_movement_allowed;

  bool humanOnPath;
  // how close another agent may be on path before the WP-follower stops to avoid collision
  float agentOnPathSafetyDistance;
  bool motionParamsSet;
  bool direction_already_computed_p;
  enum driving_direction_t driving_direction;
  double last_known_distance_to_goal;




  std::list<XYTH_COORD> waypoint_queue;


  long waitUntil; //a time to wait before moving or -1

  /** time when robot stopped for human on path */
  long pathBlockedTimestamp;
  /** time robot should wait in blocked position until human is evades */
  long blockWaitTimeSecs;

};

}

#endif /* WAYPOINTFOLLOWER_H_ */
