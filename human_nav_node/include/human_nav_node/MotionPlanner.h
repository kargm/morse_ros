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
	}
  virtual ~MotionPlanner() {}

  int init(std::string filename, bool showInterface);
  void updateInterface();
  int updPosAndFindNavTrajExec(MHP_UPD_FINDPATH &findpath_params, MHP_NAV_TRAJECTORY &result, int *report);
  int findNavTrajExec(MHP_NAV_POS &MotionCoord, MHP_NAV_TRAJECTORY &ABS_NAV_traj, int *report);
  int changeCameraPosMain(MHP_CAM_POS *cam_pos, int *report);
  int mhpPlaceAgentMain(MHP_AGENT_POSITION *addedAgent, int *report);
  int initialize_navigation();

  double grid_sampling;
  bool isInitialized;
  bool showInterface;
};

#endif /* MOTIONPLANNER_H_ */
