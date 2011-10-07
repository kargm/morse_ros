/*
 * HumanTracker.h
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#ifndef HUMANTRACKER_H_
#define HUMANTRACKER_H_


#include "Human.h"
#include "HumanHistory.h"

#define MHP_MH_SHOW_ALL_MOVEMENTS 0
#define TRACKER_MAX_HUMANS 15

namespace NHPPlayerDriver
{

class HumanTracker {

public:
  HumanTracker();
  int getHumanMaxNo();
  Human* getHumans();
  Human* getHuman(int index);
  bool isMoving(int id);
  VELOCITY getHumanVelocity(int index);
  void setLocked(int id);
  bool isLocked(int id);
  bool exists(int id);
  bool checkStoppedMoving();
  int handlePositionUpdate(float px, float py, float paz, int id);


private:
  /** an array of present humans */
  Human humans[TRACKER_MAX_HUMANS];
  HumanHistory humansHistory[TRACKER_MAX_HUMANS];
  int human_max_no;//<=MHPD_MAX_HUMANS;

};

}
#endif /* HUMANTRACKER_H_ */
