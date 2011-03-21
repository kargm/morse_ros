/*
 * MotionPlanner.h
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#ifndef MOTIONPLANNER_H_
#define MOTIONPLANNER_H_

#include <string>
#include <mhpStruct.h>

class MotionPlanner {
public:
  MotionPlanner() {
	  isInitialized = false;
	  showInterface = false;
	  grid_sampling = 0.15; // m
	}
  virtual ~MotionPlanner() {}

  int init(std::string filename, bool showInterface);
  void updateInterface();
  int updPosAndFindNavTrajExec(MHP_UPD_FINDPATH *findpath_params, int *report);
  int findNavTrajExec(MHP_NAV_POS *MotionCoord, int *report);
  int mhpPlaceAgentMain(MHP_AGENT_POSITION *addedAgent, int *report);
  int initialize_navigation();

  double grid_sampling;
  bool isInitialized;
  bool showInterface;
};

#endif /* MOTIONPLANNER_H_ */
