#include "HumanTracker.h"
#include "M3DW_Utils.h"

#include <cmath>
#include <ros/ros.h>

// how many secs a human needs to be stable before we consider him standing again after move
#define MOVING_CLASS_THRESHOLD_SECS 2
// For how much time after that we consider that the human "just stopped". // TODO: better keep track of states
#define MOVING_CLASS_WINDOW_SECS 6

// how much the sensor data XY-POS needs to deviate from last belief before we update belief
#define POS_UPDATE_THRESHOLD 0.20
// how much the sensor data Zangle-POS needs to deviate from last belief before we update belief
#define ANGLE_UPDATE_THRESHOLD 0.3

#define M3DW_STANDING 0
#define M3DW_MOVING 2

/*
 * HumanTracker.cpp
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

namespace NHPPlayerDriver {

HumanTracker::HumanTracker() {
  for (int human_i = 0; human_i < TRACKER_MAX_HUMANS ; human_i++) {
    humans[human_i].exists = FALSE;
    humans[human_i].locked = FALSE;
    humans[human_i].pose = M3DW_MOVING;
    humans[human_i].lastMoveTimeStampSecs = -1;
  }

}

/**
 * the maximal number of humans tracked, also the array size for array methods
 */
int HumanTracker::getHumanMaxNo()
{
  return TRACKER_MAX_HUMANS;
}

Human *HumanTracker::getHumans()
{
  for (int human_i = 0; human_i < TRACKER_MAX_HUMANS ; human_i++) {
     if(humans[human_i].exists == TRUE) {
       if (isMoving(human_i)) {
         humans[human_i].pose = M3DW_MOVING;
       } else {
         humans[human_i].pose = M3DW_STANDING;
       }
     }
   }
  return humans;
}

Human *HumanTracker::getHuman(int index)
{
  return &humans[index];
}

VELOCITY HumanTracker::getHumanVelocity(int index)
{
  return humansHistory[index].bodyVelocity();
}

/**
 * returns whether the robot considers the human as currently moving
 */
bool HumanTracker::isMoving(int id)
{
  if (exists(id)) {
    // return true for newly discovered humans and moved humans
    if (humans[id].lastMoveTimeStampSecs == -1 ||
        getCurrentSecs() - humans[id].lastMoveTimeStampSecs < MOVING_CLASS_THRESHOLD_SECS) {
      if (getHumanVelocity(id).speedms > 0.07 ) {
        return true;
      }
    }
  }
  return false;
}

bool HumanTracker::exists(int id)
{
  if (id >= 0 && id < getHumanMaxNo()) {
    return (humans[id].exists != 0);
  }
  return false;
}

bool HumanTracker::isLocked(int id) {
  if (exists(id) ) {
     return (humans[id].locked != 0);
   }
   return false;
}

void HumanTracker::setLocked(int id) {
  if (exists(id) ) {
    humans[id].locked = TRUE;
  }
}

/**
 * return true if human stopped moving a certain time ago
 * called regularly to trigger replanning if another agent (the human) just stopped moving (not along time later)
 */
bool HumanTracker::checkStoppedMoving()
{
  long lastMoveSecs;
  long currenttime = getCurrentSecs();
  for (int human_i = 0; human_i < TRACKER_MAX_HUMANS ; human_i++) {
    if(getHumans()[human_i].exists == TRUE && getHumans()[human_i].lastMoveTimeStampSecs > 0) {
      lastMoveSecs = currenttime - getHumans()[human_i].lastMoveTimeStampSecs;
      // TODO: better keep track of states, somehow.
      if ( (lastMoveSecs > MOVING_CLASS_THRESHOLD_SECS) &&
          (lastMoveSecs < MOVING_CLASS_THRESHOLD_SECS + MOVING_CLASS_WINDOW_SECS) ) { // if human stopped in this interval, trigger replanning
        //PLAYER_MSG0(PLAYER_ERR_DBG, "Human stopped moving.");
          // avoid constant replanning by modifying the time step TODO fix this with the above
        getHumans()[human_i].lastMoveTimeStampSecs = getHumans()[human_i].lastMoveTimeStampSecs - MOVING_CLASS_WINDOW_SECS;
//        if (MHP_MH_SHOW_ALL_MOVEMENTS) {
//      	  m3dw_setHumanPositionStanding(human_i, getHumans()[human_i].x, getHumans()[human_i].y, getHumans()[human_i].az);
//        }
        return true;
      }
    }
  }
  return false;
}

/**
 * returns TRUE if position changed, and updates knowledge about humans
 */
int HumanTracker::handlePositionUpdate(float px, float py, float paz, int id)
{
  if (id < 0 || id > TRACKER_MAX_HUMANS) { // should never happen
    ROS_ERROR("Error, bad ID %d", id);
    return 0;
  }

  bool xyChange = false;
  bool angleChange = false;

  //yaw is for nodding, pitch is for shaking
  if (humansHistory[id].addHeadIsValid(px, py, 1.7, /*yaw*/  0, /*pitch*/ paz)) {
      // have 5cm tolerance for human movement, for regular updates of same position
      if (fabs(getHumans()[id].x - px) > POS_UPDATE_THRESHOLD ||
          fabs(getHumans()[id].y - py) > POS_UPDATE_THRESHOLD) {
          xyChange = true;
          getHumans()[id].lastMoveTimeStampSecs = getCurrentSecs();
          humans[id].locked = FALSE;
          //    PLAYER_MSG4(PLAYER_ERR_DBG, "Human moved from %f,%f to %f,%f.",getHumans()[id].x,getHumans()[id].y,px, py);
          //PLAYER_MSG2(PLAYER_ERR_DBG, "Updated human %d timestamp %d", id, getHumans()[id].lastMoveTimeStampSecs);
      }

      if (fabs(NORMALIZE(getHumans()[id].az - paz)) > ANGLE_UPDATE_THRESHOLD) {
          angleChange = true;
          //PLAYER_MSG2(PLAYER_ERR_DBG, "Human moved angle from %f to %f.",getHumans()[id].az,paz);
      }

      if (xyChange || angleChange) {

          getHumans()[id].x = px;
          getHumans()[id].y = py;
          getHumans()[id].az = paz;
          getHumans()[id].exists = TRUE;


          //    // TODO this should not happen here, but in the driver code or so
          //    if (MHP_MH_SHOW_ALL_MOVEMENTS) {
          //      m3dw_setHumanPositionMoving(id, px, py, paz);
          //    }

          return TRUE;
      }
  }

  return FALSE;
}

}
