/*
 * WPFollowerParams.h
 *
 *  Created on: Oct 19, 2010
 *      Author: kruset
 */

#ifndef WPFOLLOWERPARAMS_H_
#define WPFOLLOWERPARAMS_H_

namespace NHPPlayerDriver {
  /**
   * structure capturing the essential parameters of waypoint-following agents
   */
  typedef struct STRUCT_WP_FOLLOW_PARAMS{
    // rad (the bigger, the earlier robot will move and turn in parallel
    double trans_angle_range;
    // how well the initial turn angle must be matched before the robot can start moving
    double initial_trans_angle_range;
    // in decimeter/s ?
    double max_trans_vel;
    // how much speed allowed relative to distance to target
    //(e.g. kp=0.9: if 1 meter left: 0.9m/s, if 0.5 meter left, 0.45m/s)
    double pid_trans_kp;

    // maximal rotation speed deg/s
    double max_rot_vel;
    // how much to increase turn speed, propotional gain (per second, per cycle?)
    double wp_max_rot_vel;

    // how much to increase turn speed, propotional gain
    double pid_rot_kp;
    // how close to the final goal we must get before stopping and succeeding
    double success_distance;
    // how close to move towards a waypoint before turning to the next one
    double wp_success_distance;
    // tolerance to goal angles
    double az_success_distance;
  } WP_FOLLOW_PARAMS;

  // to be implemented by robot specific cpp file
  WP_FOLLOW_PARAMS getRobotMotionParams();
}

#endif /* WPFOLLOWERPARAMS_H_ */
