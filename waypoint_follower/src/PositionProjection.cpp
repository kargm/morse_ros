/*
 * projects position of agents in the future based on sensor readings etc.
 *
 * TODO: Projection for constant acceleration and curvature model (bounded min/max velo)
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#include "PositionProjection.h"

#include <algorithm>
#include <cmath>
#include <M3DW_Utils.h>


namespace NHPPlayerDriver {


  MOTION_PROJECTION_PARAMS getDefaultMotionProjectionParams() {
    MOTION_PROJECTION_PARAMS params;
    params.projectionSteps=0.5;
    return params;
  }

  /**
   * projects position of object or agent according to linear model of motion
   */
  XYTH_COORD linearProjection(XYTH_COORD currentPose, VELOCITY velocity, double timeSecs) {
    XYTH_COORD result;
    // simply add velocity onto current pose w.r.t. given time
    result.x = currentPose.x + velocity.speedxms * (timeSecs );
    result.y = currentPose.y + velocity.speedyms * (timeSecs );
    result.th = currentPose.th;
    return result;
  }

  /**
   * projects position agent according to minimum jerk model of motion
   */
  XYTH_COORD minJerkProjection(XYTH_COORD currentPose, double acceleration, VELOCITY velocity, double timems) {
    XYTH_COORD result;
    // acceleration tells us wherther we want to reach a higher or a lower velocity
    // minimum jerk models tells us how acceleration will change.
    // acceleration tells us how velocity will change.
    return result;
  }


  bool humanInFront(XYTH_COORD currentRobotPose, XYTH_COORD humanPose) {
//    if (DISTANCE2D(currentRobotPose.x, currentRobotPose.y, humanPose.x, humanPose.y) > 0.5) {
//      return false; // bug somehwere, human sees itself as obstacle
//    }
    double angle2human = atan2(humanPose.y - currentRobotPose.y, humanPose.x - currentRobotPose.x);
    double angle_deviation = angle2human - currentRobotPose.th;
    while (angle_deviation < -M_PI) {
         angle_deviation = M_2PI + angle_deviation;
       }
      while(angle_deviation > M_PI) {
         angle_deviation = M_2PI - angle_deviation;
       }
//    PLAYER_MSG3(PLAYER_ERR_ERR, "Human at  %f, robot pointing to %f, deviation %f", angle2human, currentRobotPose.th, angle_deviation);

       return fabs(angle_deviation) < M_PI_2;
  }

  /**
   * complex function that takes into account several humans with their current velocities,
   * projects their position linearly in time, projects the robot position along a path given
   * certain motion parameters (e.g. max robot speed) in time, and returns true if the projection
   * results in a conflict within the given time span.
   */
  bool checkPosesInConflict( // TODO pass pose histories instead, or agents
      XYTH_COORD* humanPoses,
      VELOCITY* humanVelocities,
      int num_humans,
      XYTH_COORD currentRobotPose,
      VELOCITY robotVelocity,
      std::list<XYTH_COORD> *pathWaypoints,
      double timeSpan,
      double collision_distance,
      WP_FOLLOW_PARAMS params) {

//    long realtime = getCurrentMillisecs();
    XYTH_COORD humanPredPose;
    double relativeTimeSecs;

    MOTION_PROJECTION_PARAMS proj_params = getDefaultMotionProjectionParams();


    // TODO: detect earlier that speed reduction helps!
    // adapt robot speed to upcoming turn


    // check for every certain space interval
    for (relativeTimeSecs = 0.0; relativeTimeSecs < timeSpan; relativeTimeSecs += proj_params.projectionSteps / (params.reduced_trans_vel / 10)) {
        XYTH_COORD predRobotPose = pathProjection(currentRobotPose, robotVelocity, pathWaypoints, relativeTimeSecs, params);
        //        MSG6(ERR_ERR,"position %f, %f, Time %d Prediction %f %f %d\n", currentRobotPose.x, currentRobotPose.y, realtime, predRobotPose.x, predRobotPose.y, realtime + ((long) round(relativeTimeSecs * 1000)));
        for (int i = 0; i < num_humans; ++i) {
          if (humanInFront(currentRobotPose, humanPoses[i])) {
            //  {TODO discard humans if outside possible range
            humanPredPose = linearProjection(humanPoses[i], humanVelocities[i], relativeTimeSecs);
            // TODO: include uncertainty in collision distance
            if (DISTANCE2D(humanPredPose.x, humanPredPose.y, predRobotPose.x, predRobotPose.y) < collision_distance) {
              return true;
            } else {
            }
          }
        }
    }
    return false;
  }

  /**
   * projects position of robot on given path, if robot is reasonably
   * close to some path waypoint, according to a model of robot behavior
   */
  XYTH_COORD pathProjection(XYTH_COORD currentPose,
      VELOCITY velocity,
      std::list<XYTH_COORD> *pathWaypoints,
      double time,
      WP_FOLLOW_PARAMS params) {

    XYTH_COORD resultPose = currentPose;
    // we need a model of the robot's local planner and control loop to predict where the robot will be.
    // For this to work, we assume the local planner has to stick close visiting the waypoints in turn
    // non-holonomic turn-move-turn models are as possible as are holonomic motions

    // to allow for greatest flexibility of use, it could have been great to actually use the local planner
    // here rather than trying to remodel it. However as a local planner often has state, this is not easy to realize

    // So what we assume is that the robot moves down the given path at a certain velocity, taking extra time for angles.

    double min_distance = 100000;
    double loop_distance;
    double next_angle, diff_angle, rotate_angle;
    std::list<XYTH_COORD>::iterator it;

    // we assume that the first local minimum distance to the robot is the point
    // on the path the robot follows, this is not strictly theoretically true.
    for (it = pathWaypoints->begin();
            it != pathWaypoints->end();
            it++) {
           loop_distance = DISTANCE2D(currentPose.x, currentPose.y, it->x, it->y);
        if (loop_distance < min_distance) {
            min_distance = loop_distance;
        } else {
            break;
        }
    }

     double expectedVelocity = std::min(velocity.speedms, params.reduced_trans_vel * 10); // TODO: change velocity over time

    // loop while projection time is left:
    while (time > 0 && it != pathWaypoints->end()) {
        loop_distance = DISTANCE2D(resultPose.x, resultPose.y, it->x, it->y);
        // Then we find the waypoint towards which the robot should head next (can be the current one, the next, or even a further one)
        while (it != pathWaypoints->end() &&
            params.wp_success_distance > loop_distance) {
            it++;
            if (it != pathWaypoints->end()) {
                loop_distance = DISTANCE2D(resultPose.x, resultPose.y, it->x, it->y);
            } else {
                loop_distance = -1;
            }
        }
        if (loop_distance == -1) {
            // we must be close enough to the goal
            resultPose.x = it->x;
            resultPose.y = it->y;
            break;
        }

        // then we find out how much the robot has to turn until that waypoint.
        next_angle = NORMALIZE(atan2(it->y - resultPose.y, it->x - currentPose.x));
        diff_angle = fabs(NORMALIZE(next_angle - resultPose.th));
        // if we need to turn, substract the time needed for turning, position the robot in correct angle (reduce if time runs out)
        if (diff_angle > params.trans_angle_range) {
            rotate_angle = fabs(NORMALIZE(params.trans_angle_range - resultPose.th));
            time = time - rotate_angle / DTOR(params.reduced_trans_vel);
            resultPose.th = next_angle; // assume robot has turned to next angle in the meantime
        }

        //   if projection time is left: move robot forward until close enough to next waypoint, substract time estimated for that (reduce if time runs out)
        if (time > 0) {
            if (time >= loop_distance / expectedVelocity) {
              time = time - loop_distance / expectedVelocity;
              resultPose.x = it->x;
              resultPose.y = it->y;
           } else {
               // less time left than needed to next wp, proportional rest
               double rest = time / ( loop_distance / expectedVelocity);
               time = 0;
               resultPose.x = resultPose.x + rest * (it->x - resultPose.x) ;
               resultPose.y = resultPose.y + rest * (it->y - resultPose.y);
           }
        }
    }
    // return the final projected location of the robot.

    return resultPose;
  }

}
