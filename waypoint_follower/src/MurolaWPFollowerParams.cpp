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
  params.trans_angle_range = 0.4;        // rad
  params.initial_trans_angle_range = 0.15;  // deg (the bigger, the earlier robot will move and turn in parallel
  params.max_trans_vel = 6;             // dm/s, max speed
  params.pid_trans_kp = 0.4;
  params.max_rot_vel = 40.0;              // deg/s
  params.wp_max_rot_vel = 55;
  params.pid_rot_kp = 1.8;

  params.success_distance = 0.20;         // m tolerance for last waypoint
  params.wp_success_distance = params.success_distance * 1.5; // tolerance for waypoints
  params.az_success_distance = 0.15;     // az tolerance to goal az
  return params;
}
}
