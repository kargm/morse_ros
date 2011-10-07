/*
 * RobotHistory.h
 *
 *  Created on: Aug 9, 2010
 *      Author: kruset
 */

#ifndef RobotHistory_H_
#define RobotHistory_H_

#include "JumpFilterQueue.h"
#include "M3DW_Utils.h"

namespace NHPPlayerDriver {

class RobotHistory {
  // queue for agent head
  PoseHistory headHist;


public:
  RobotHistory();
  virtual ~RobotHistory();
  int addHeadIsValid(double x, double y, double z, double yaw, double pitch);
  int headNumReadings();
  double lastHeadDiff();
  VELOCITY bodyVelocity();
};



}
#endif /* RobotHistory_H_ */
