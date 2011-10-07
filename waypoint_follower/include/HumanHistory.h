/*
 * HumanHistory.h
 *
 *  Created on: Aug 9, 2010
 *      Author: kruset
 */

#ifndef HUMANHISTORY_H_
#define HUMANHISTORY_H_

#include "PoseHistory.h"
#include "M3DW_Utils.h"

namespace NHPPlayerDriver {




typedef enum STRUCT_HUMAN_STATE_ENUM{
  // must be the same as BT_... constants in Move3d hri_planner
  STANDING = 0,
  SITTING = 1,
  MOVING = 2,
  STANDING_TRANSPARENT = 3// if planner may plan through this human
} HUMAN_STATE_ENUM;

class HumanHistory {
  // queue for agent head
  PoseHistory headHist;


//  // queue for agent left hand
//  JumpFilterQueue<AGENT_HAND_READING, HUMAN_HISTORY_LENGTH, handReadingDiff> lHandQueue;
//  // queue for agent right hand
//  JumpFilterQueue<AGENT_HAND_READING, HUMAN_HISTORY_LENGTH, handReadingDiff> rHandQueue;


public:
  HumanHistory();
  virtual ~HumanHistory();
  int addHeadIsValid(double x, double y, double z, double yaw, double pitch);
  int headNumReadings();
  double lastHeadDiff();
  VELOCITY bodyVelocity();

  HUMAN_STATE_ENUM headState();

};
}
#endif /* HUMANHISTORY_H_ */
