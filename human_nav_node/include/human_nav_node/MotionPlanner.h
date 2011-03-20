/*
 * MotionPlanner.h
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#ifndef MOTIONPLANNER_H_
#define MOTIONPLANNER_H_

#include <string>

class MotionPlanner {
public:
  MotionPlanner();
  virtual ~MotionPlanner();

  int init(std::string filename, bool showInterface);

  bool isInitialized;
};

#endif /* MOTIONPLANNER_H_ */
