/*
 * Human.h
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#ifndef HUMAN_H_
#define HUMAN_H_

/**
  * A structure to capture all known / relevant humans in the environment
  */
typedef struct Human
 {
   double x;
   double y;

   double az; // head rotation

   int pose; // position (standing sitting moving) as int
   int locked; // whether human will not move out of the way
   int exists; // 0 if not
   long lastMoveTimeStampSecs; // last time we got a move, used to determine if human is moving
   long lastStandTimeStampSecs; // last time we got no move, used to determine if human is moving
 } Human;


#endif /* HUMAN_H_ */
