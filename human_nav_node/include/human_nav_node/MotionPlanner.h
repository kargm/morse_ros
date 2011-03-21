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
  MotionPlanner() {
	  isInitialized = false;
	  showInterface = false;
	}
  virtual ~MotionPlanner() {}

  int init(std::string filename, bool showInterface);
  void updateInterface();

  bool isInitialized;
  bool showInterface;
};

#endif /* MOTIONPLANNER_H_ */
