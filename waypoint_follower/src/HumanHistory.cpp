/*
 * HumanHistory.cpp a class that takes in multiple sensor readings for different body parts, filters for
 * erroneous jumps, and performs analysis on the buffered histories of readings (pose and gesture recognition, prediction)
 *
 *  Created on: Aug 9, 2010
 *      Author: kruset
 */

#include "HumanHistory.h"

#include "JumpFilterQueue.h"

#include <cmath>
#include <stdio.h>

// if the head is below that heigt, we consider the human sitting.
#define HUMAN_DEFAULT_SITTING_HEAD_HEIGHT 1.6
// sensor of maximally what time in millisecs to consider for motion detection
#define VELOCITY_EST_HORIZON 3000
// how many sensor values to keep in FIFO buffer
#define HUMAN_HISTORY_LENGTH 15
// human moving below that value in (m/s) is considered standing
#define MINIMUM_MOTION_VELOCVITY 0.3

namespace NHPPlayerDriver {

HumanHistory::HumanHistory() {
  // init with constructor values
  headHist = PoseHistory();
}

HumanHistory::~HumanHistory() {
}


/**
 * yaw is for nodding, pitch is for shaking
 */
int HumanHistory::addHeadIsValid(double x, double y, double z, double yaw, double pitch)
{
  return headHist.addPoseIsValid(x, y, z, yaw, pitch);
}

//int HumanHistory::addLeftHandIsValid(double x, double y, double z)
//{
//  AGENT_HAND_READING r;
//  r.hand.x =x;
//  r.hand.x =y;
//  r.hand.x =z;
//  return lHandQueue.addIsValid(r);
//}
//
//int HumanHistory::addRightHandIsValid(double x, double y, double z)
//{
//  AGENT_HAND_READING r;
//  r.hand.x =x;
//  r.hand.x =y;
//  r.hand.x =z;
//  return rHandQueue.addIsValid(r);
//}

/**
 * computes the sensor history to check whether they indicate that the human is moving.
 * considers that the first element of the history is the current position
 * returns a state of posture or motion
 */
HUMAN_STATE_ENUM HumanHistory::headState()
{
  VELOCITY vel = headHist.velocity();
    if (vel.speedms > MINIMUM_MOTION_VELOCVITY) { // magic heuristic adapted to LAAS Mocap system
        return MOVING;
    } else {
        //   last reading in sitting height
        if (headHist.numReadings() > 0 && headHist.validPose().pose.z < HUMAN_DEFAULT_SITTING_HEAD_HEIGHT) {
            return SITTING;
        } else {
            return STANDING;
        }
    }
}

VELOCITY HumanHistory::bodyVelocity() {
  return headHist.velocity();
}

int HumanHistory::headNumReadings() {
  return headHist.numReadings();
}

double HumanHistory::lastHeadDiff() {
  return headHist.lastDiff();
}

}// end namespace
