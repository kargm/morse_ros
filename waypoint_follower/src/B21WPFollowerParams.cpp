#include "WPFollowerParams.h"

/*
 * WPFollowerParams.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: kruset
 */

namespace NHPPlayerDriver {

WP_FOLLOW_PARAMS params;

WP_FOLLOW_PARAMS getRobotMotionParams() {

  // values that work in b21 gazebo simulation
  params.trans_angle_range = 0.5;        // rad
  params.initial_trans_angle_range = 0.9;  //(the bigger, the earlier robot will move and turn in parallel
  params.max_trans_vel = 2;             // dm/s, max speed
  params.pid_trans_kp = 0.4;
  params.max_rot_vel = 30.0;              // deg/s
  params.wp_max_rot_vel = 35;
  params.pid_rot_kp = 1.1; // potential gain of roation

  params.success_distance = 0.02;         // m tolerance for last waypoint
  params.wp_success_distance = params.success_distance * 8; // tolerance for waypoints
  params.az_success_distance = 0.05;     // az tolerance to goal az
  return params;
}
}
