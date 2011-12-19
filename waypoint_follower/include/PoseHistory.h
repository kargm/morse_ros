/*
 * PoseHistory.h
 *
 *  Created on: Aug 9, 2010
 *      Author: kruset
 */

#ifndef POSEHISTORY_H_
#define POSEHISTORY_H_

#include "JumpFilterQueue.h"
#include "M3DW_Utils.h"

namespace NHPPlayerDriver {

// sensor of maximally what time in millisecs to consider for motion detection
#define VELOCITY_EST_HORIZON 2000
// how many sensor values to keep in FIFO buffer
#define HISTORY_LENGTH 15


typedef struct STRUCT_LIMP_6D_COORD{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} LIMP_6D_COORD;


/**
 * one reading of several agent headpositions
 */
typedef struct STRUCT_POSE_READING {
  LIMP_6D_COORD pose;
  unsigned long sensor_time_head;
} POSE_READING;


double readingDiff(POSE_READING a , POSE_READING b);

class PoseHistory {
  JumpFilterQueue<POSE_READING, HISTORY_LENGTH, readingDiff> poseQueue;


public:
  PoseHistory();
  virtual ~PoseHistory();
  int addPoseIsValid(double x, double y, double z, double yaw, double pitch);
  int numReadings();
  POSE_READING validPose();
  void valid2DPose(XYTH_COORD*);
  double lastDiff();
  VELOCITY velocity();
};



VELOCITY compute_current_motion(JumpFilterQueue<POSE_READING, HISTORY_LENGTH, readingDiff> history);
}
#endif /* POSEHISTORY_H_ */
