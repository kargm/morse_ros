/*
 * JumpFilterQueuer.h
 *
 *  Created on: Aug 8, 2010
 *      Author: kruset
 */

#ifndef NOFIMOFI_H_
#define NOFIMOFI_H_

//#ifndef TRUE
//#define TRUE  1
//#define FALSE 0
//#endif

#include "SensorReadingQueue.h"

/*
 * JumpFilterQueue.h
 *
 * keeps a queue of history readings of size BUFN, adds new readings to queue,
 * tries to be robust against jumps using heuristic methods.
 * This assumes readings are expected to have a certain difference from each other,
 * it cannot cope with fast motions producing readings spread apart far.
 * In such cases it will stall until values stabilize.
 *
 * For flexible usage, it is recommended to use a diff emasure invariant
 * of sensor reading frequencey, e.g. velocity instead of distance.
 *
 * Uses the provided diff function to compare sensor readings,
 * and considers any difference greate than the threshold constructor
 * value as potential noise. IN such a case, the last valid value will not change, only
 * after <validjump> more readings sufficiently close together will it accept the jump.
 *
 * validates jumps after VALIDJUMP readings with diff > THRES
 */
template<typename T, int BUFN, double (*T_diff)(T, T)> class JumpFilterQueue {
  SensorReadingQueue<T, BUFN> sensorQueue;
  int jumpCounter;
  double diffThreshold;
  int validJumpThres;

public:
  JumpFilterQueue(double threshold, int validjump)
  :
    jumpCounter(0), diffThreshold(threshold), validJumpThres(validjump)
  {
  }

  JumpFilterQueue()
  :
    jumpCounter(0), diffThreshold(1), validJumpThres(5)
  {
  }

  void initQueue(double newDiffThreshold, int newValidJumpThres)
  {
    diffThreshold = newDiffThreshold;
    validJumpThres = newValidJumpThres;
  }

  /**
   * returns the queue, TODO: Do not expose
   */
  SensorReadingQueue<T, BUFN> getQueue(){
    return sensorQueue;
  }

  /**
   * returns the nth valid element from front, meaning if we push a, b, c, d, (d not validated), validElementAtR(0) => c;validElementAtR(1) => b;
   * always returns an element from the backing array, might return invalid values though.
   */
  T validElementAtR(int i)
  {
    return sensorQueue.validElementAtR(i);
  }

  /**
   * array operator referencing valid elements for convenience
   */
  T operator[](int i) {

    return validElementAtR(i);

  }

  int sizeValid() const
  {
    return sensorQueue.sizeValid();
  }

  /**
   * adds a new reading with difference diff (TODO use template with diff function parameter)
   * returns TRUE if the new one could be validated
   */
  bool addIsValid(T newReading)
  {
    double diffToLastValid, diffToLast;
    if (sensorQueue.size() == 0) {
      diffToLastValid = 0;
    } else {
      diffToLastValid = (*T_diff)(newReading, validElementAtR(0));
    }

    if(diffToLastValid > diffThreshold ) {
      //there is a big jump, might be a fluke, need to perceive it several times before believing it.
      if (jumpCounter > 0) {
        //last reading was already a potential jump, so check whether this stabilizes
        diffToLast = (*T_diff)(newReading, sensorQueue.elementAtR(0));
        if (diffToLast < diffThreshold ) {
          jumpCounter++;
        } else {
          // did not stabilize
          sensorQueue.purgeNonValidated();
          jumpCounter = 1;
        }
      } else {
        jumpCounter = 1;
      }
    } else {
      // previous jumped values are incorrect after all
      if (jumpCounter > 0) {
        //discard all that were pushed nut not yet validated
        sensorQueue.purgeNonValidated();
      }
      jumpCounter = 0;
    }


    /*
     *  we push all new readings, even the jumps,
     *  so that later we can recognize a moving or standing state
     *  immediately after we validated a jump
     */
    sensorQueue.push(newReading); //pushed but not yet validated

    // if no jump suspicion validate. If confirmed jump, purge all previous values, switch to new ones
    if (jumpCounter == 0) {
      sensorQueue.validateAll();
    } else if (jumpCounter > validJumpThres) {
      // since we jump, the previously VALID values before the jump are outdated
      sensorQueue.purgeValids();
      // all values since the jump now become valid
      sensorQueue.validateAll();
      jumpCounter = 0;
    }

    if (jumpCounter == 0) {
      // last value is to be believed
      return true;
    }
    return false;
  }

};




#endif /* NOFIMOFI_H_ */
