/*
 * PoseHistory.cpp a class that takes in 3d 6DOF sensor readings, filters for
 * erroneous jumps, and performs gemotric analysis on a buffered history of readings (velocity, acceleration)
 *
 *  Created on: Aug 9, 2010
 *      Author: kruset
 */

#include "PoseHistory.h"
#include <cmath>
#include <stdio.h>

namespace NHPPlayerDriver {

PoseHistory::PoseHistory() {
  // init with constructor values
  poseQueue = JumpFilterQueue<POSE_READING, HISTORY_LENGTH, readingDiff>(2, 5);
}

PoseHistory::~PoseHistory() {
}

/**
 * function to give the difference between head readings as one double, for the sensor history queue
 */
double readingDiff(POSE_READING a , POSE_READING b) {
  // distance as velocity required to go from one point to other
  double updateDistDiff = DISTANCE3D(a.pose.x, a.pose.y, a.pose.z,
      b.pose.x, b.pose.y, b.pose.z); // meters

//  long timeDiff = fabs(a.sensor_time_head - b.sensor_time_head); //millisecs
//  double velo = 100 * (updateDistDiff / timeDiff); // in meter/sec

  return updateDistDiff;
}

/**
 * yaw is for nodding, pitch is for shaking
 */
int PoseHistory::addPoseIsValid(double x, double y, double z, double yaw, double pitch)
{
  POSE_READING newReading;

  newReading.pose.x = x;
  newReading.pose.y = y;
  newReading.pose.z = z;
  newReading.pose.yaw = yaw;
  newReading.pose.pitch = pitch;
  newReading.pose.roll= 0.0;
  newReading.sensor_time_head = getCurrentMillisecs();
  return poseQueue.addIsValid(newReading);
}


int PoseHistory::numReadings() {
  return poseQueue.sizeValid();
}

double PoseHistory::lastDiff() {
  if (poseQueue.sizeValid() > 1) {
    return readingDiff(poseQueue[0], poseQueue[1]);
  } else {
    return 0;
  }
}


VELOCITY PoseHistory::velocity() {
  return compute_current_motion(poseQueue);
}

/**
 * The last validated reading (jumps filtered out)
 */
POSE_READING PoseHistory::validPose() {
  return poseQueue[0];
}

/**
 * convenience method transforming 6DOF pose in 2d format
 */
void PoseHistory::valid2DPose(XYTH_COORD* target) {
  target->x = validPose().pose.x;
  target->y = validPose().pose.y;
  target->th = validPose().pose.pitch;
}

VELOCITY compute_current_motion(JumpFilterQueue<POSE_READING, HISTORY_LENGTH, readingDiff> history) {
  VELOCITY vel;

  double rdistx = 0, rdisty = 0, velox=0, veloy=0;
  long timeDeltaMs = VELOCITY_EST_HORIZON + 1; // to step into loop once
  long currentTime = getCurrentMillisecs();
  double sumVelox=0, sumVeloy=0;
  int i = 0;

  vel.heading_az = 0;
  vel.speedms = 0;
  vel.speedxms = 0;
  vel.speedyms = 0;

  // we want to get an estimate of the motion using a mere linear model
  //(TODO professional 2d time series analysis)
  // the valid sensor readings have time stamps and may not be in regular time intervals.
  // so we take for each delta in sensor readings the velocity in x and y direction, and average over those.
  if (history.sizeValid() > 2) {
      for (i = 0; i < history.sizeValid() - 1 && (currentTime - history[i].sensor_time_head) < VELOCITY_EST_HORIZON; ++i) {
        // smart array operator, not an actual array! [0] is always most recent value
//        dist = DISTANCE2D(history[i].head.x,
//            history[i].head.y,
//            history[i+1].head.x,
//            history[i+1].head.y);
        rdistx = history[0].pose.x - history[i + 1].pose.x;
        rdisty = history[0].pose.y - history[i + 1].pose.y;
        timeDeltaMs = history[0].sensor_time_head - history[i + 1].sensor_time_head;

        velox = 1000 * (rdistx / timeDeltaMs);
        veloy = 1000 * (rdisty / timeDeltaMs);

        sumVelox += velox;
        sumVeloy += veloy;

      }
      vel.heading_az = atan2(history[0].pose.y - history[i].pose.y, history[0].pose.x - history[i].pose.x);
      vel.speedxms = sumVelox / i;
      vel.speedyms = sumVeloy / i;
      vel.speedms = (sqrt(pow(sumVelox, 2) + pow(sumVeloy, 2)))/ i;
  }

  return vel;
}
}// end namespace
