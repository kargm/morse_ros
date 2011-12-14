/*
 * projects position of agents in the future based on sensor readings etc.
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#ifndef POSPROJECT_H_
#define POSPROJECT_H_

#include "M3DW_Utils.h"
#include "WPFollowerParams.h"
#include <list>



namespace NHPPlayerDriver {

#define M_2PI  6.28318531


  XYTH_COORD linearProjection(XYTH_COORD currentPose, VELOCITY velocity, double time);
  XYTH_COORD pathProjection(XYTH_COORD currentPose, VELOCITY velocity, std::list<XYTH_COORD> *pathWaypoints, double time, WP_FOLLOW_PARAMS params);

  /**
    * structure capturing the essential parameters of waypoint-following agents
    */
   typedef struct STRUCT_MOTION_PROJECTION_PARAMS {
     // max in meters distance between projection step
     double projectionSteps;
   } MOTION_PROJECTION_PARAMS ;

   MOTION_PROJECTION_PARAMS getDefaultMotionProjectionParams();


  double checkPosesInConflict(
        XYTH_COORD* humanPoses,
        VELOCITY* humanVelocities,
        int no_humans,
        XYTH_COORD currentRobotPose,
        VELOCITY robotVelocity,
        std::list<XYTH_COORD> *pathWaypoints,
        double timeSpan,
        double collision_distance,
        WP_FOLLOW_PARAMS params);
}

#endif /* POSPROJECT_H_ */
