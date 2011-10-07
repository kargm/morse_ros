

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
  WaypointFollower(WP_FOLLOW_PARAMS params);
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
  void waitInPosition(long seconds);

public:  WP_FOLLOW_PARAMS motionParams;

//private:
  //bool isHumanOnPath(float safetyDistance);


private: void turnVelocityInGoal(VELOCITY_COMMAND & cmd);
private: void updateGoalForNextWaypoint();
private: void updateVelocityBetweenWaypoints(VELOCITY_COMMAND & cmd, double current_distance_to_goal);
private: void updateDrivingDirection(double angle_towards_goal_absolute, double & current_distance_to_goal, double & start_driving_forward_turning_angle, double & start_driving_backward_turning_angle);
private: int prunePlan();
//private:	void changeGoalPosition(float x, float y, float az, double max_vel, float pid_rot_kp, double max_rot_vel );

//  // the last percept
private: float x, y, az;
// the goal position
private: float gx, gy, gaz, final_gaz;

private: bool position_initialized;
private: bool goal_ready;
private: bool robot_movement_allowed;

private: bool humanOnPath;
// how close another agent may be on path before the WP-follower stops to avoid collision
private: float agentOnPathSafetyDistance;

private: bool direction_already_computed_p;
private: enum driving_direction_t driving_direction;
private: double last_known_distance_to_goal;




private: std::list<XYTH_COORD> waypoint_queue;


private: long waitUntil; //a time to wait before moving or -1

/** time when robot stopped for human on path */
private: long pathBlockedTimestamp;
/** time robot should wait in blocked position until human is evades */
private: long blockWaitTimeSecs;

};

}

#endif /* WAYPOINTFOLLOWER_H_ */
