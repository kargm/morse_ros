#include "HumanHistory.h"
#include "PoseHistory.h"
#include "PositionProjection.h"


#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>

//#include <portLib.h>
//#include <h2initGlob.h>
//#include <taskLib.h>

//#include <server/mhpMsgLib.h>
//
//#include <server/mhpPosterLib.h>
//#include "mhpPrint.h"
//#include "mhpScan.h"

#include <math.h>

#ifndef OK
#define OK 0
#endif
#ifndef ERROR
#define ERROR (-1)
#endif


namespace NHPPlayerDriver {


/**
 * test function
 */
int testLinearprojection()
{
  int res = 0;
  HumanHistory hist;
  XYTH_COORD currentHumPose;
  XYTH_COORD predHumPose;

  hist.addHeadIsValid(2.1, 4.1, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.5, 3.1, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.3, 2.11, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.4, 1.3, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.5, 0.4, 0, 0, 0);


  VELOCITY vel = hist.bodyVelocity();
  currentHumPose.x = 2.5;
  currentHumPose.y = 0.4;
  currentHumPose.th = vel.heading_az;


  printf("Velocity: %f x:%f y:%f heading:%f\n", vel.speedms, vel.speedxms, vel.speedyms, vel.heading_az);

  // asserts
  if (vel.speedms < 0.7 || vel.speedms > 1) {res = ERROR; }
  if (vel.speedxms < 0.03 || vel.speedxms > 0.1) {res = ERROR; }
  if (vel.speedyms < -1 || vel.speedyms > -0.7) {res = ERROR; }

  predHumPose = linearProjection(currentHumPose, vel, 1.0);
  printf("Predpose: 1 x:%f y:%f heading:%f\n", predHumPose.x, predHumPose.y, predHumPose.th);
  if (predHumPose.x  < 2.5 || predHumPose.x > 2.7) {res = ERROR; }
  if (predHumPose.y < -0.8 || predHumPose.y > -0.4) {res = ERROR; }


  predHumPose = linearProjection(currentHumPose, vel, 2.0);
  printf("Predpose: 2 x:%f y:%f heading:%f\n", predHumPose.x, predHumPose.y, predHumPose.th);
  predHumPose = linearProjection(currentHumPose, vel, 3.0);
  printf("Predpose: 3 x:%f y:%f heading:%f\n", predHumPose.x, predHumPose.y, predHumPose.th);

//  int res = m3dw_initClient("random");
//  res = m3dw_setupMhpInitState("xmlrpc", "/usr/proj/openrobots/examples/examples/KiboKitchen/kibo.p3d");
//
//
//  AbsoluteTrajectory waypoints;
//  res = m3dw_requestPathTo(1.5, 1.5, 0, 3, 3, 0, NULL, 0, &waypoints, 0);
//
//  if (res != 0) {
//    printf("Error:%d\n", res);
//  } else {
//    printf("Number of waypoints: %d \n", waypoints.numberOfSegments);
//  }
//
//  m3dw_destroyClient();

  return res;
}

/**
 * test function
 */
int testPathProjection()
{
  int res = 0;
  XYTH_COORD currentpose;
  XYTH_COORD predpose;
  VELOCITY vel;
  std::list<XYTH_COORD> path;
  XYTH_COORD wp;

  int numberOfSegments = 10;

  double wps[] = {
      // x, y, th (don't care?)
      0.0, 0.0, 0.0,
      0.2, 0.0, 0.0,
      0.4, 0.2, 0.0,
      0.4, 0.4, 0.0,
      0.4, 0.6, 0.0,
      0.6, 0.8, 0.0,
      0.8, 1.0, 0.0,
      1.0, 1.0, 0.0,
      1.2, 1.0, 0.0,
      1.4, 1.2, 0.0 };

  currentpose.x = 0.2;
  currentpose.y = 0.1;
  currentpose.th = 0.5;

  for (int i = 0; i < numberOfSegments ; ++i) {
      wp.x = wps[3 * i];
      wp.y = wps[3 * i + 1];
      wp.th = wps[3 * i + 2];
      path.push_back(wp);
  }

  WP_FOLLOW_PARAMS params = getRobotMotionParams();

  params.max_trans_vel = 0.5;
  vel.speedms = 0.6;
  vel.heading_az = 0.5;

  predpose = pathProjection(currentpose, vel, &path, 0.5, params);
  printf("Predpose: 0.5 x:%f y:%f heading:%f\n", predpose.x, predpose.y, predpose.th);
  predpose = pathProjection(currentpose, vel, &path, 1.0, params);
  printf("Predpose: 1 x:%f y:%f heading:%f\n", predpose.x, predpose.y, predpose.th);
  predpose = pathProjection(currentpose, vel, &path, 2.0, params);
  printf("Predpose: 2 x:%f y:%f heading:%f\n", predpose.x, predpose.y, predpose.th);
  predpose = pathProjection(currentpose, vel, &path, 3.0, params);
  printf("Predpose: 3 x:%f y:%f heading:%f\n", predpose.x, predpose.y, predpose.th);
  predpose = pathProjection(currentpose, vel, &path, 4.0, params);
  printf("Predpose: 4 x:%f y:%f heading:%f\n", predpose.x, predpose.y, predpose.th);



  return res;
}


int testPoseConflict()
{
  int res = 0;
  XYTH_COORD currentRobotPose;
  XYTH_COORD predRobotPose;
  VELOCITY robotVel;
  std::list<XYTH_COORD> path;
  XYTH_COORD wp;
  HumanHistory hist;

  int numberOfSegments = 10;

  double wps[] = {
      // x, y, th (don't care?)
      0.0, 0.0, 0.0,
      0.2, 0.0, 0.0,
      0.4, 0.2, 0.0,
      0.4, 0.4, 0.0,
      0.4, 0.6, 0.0,
      0.6, 0.8, 0.0,
      0.8, 1.0, 0.0,
      1.0, 1.0, 0.0,
      1.2, 1.0, 0.0,
      1.4, 1.2, 0.0 };

  currentRobotPose.x = 0.2;
  currentRobotPose.y = 0.1;
  currentRobotPose.th = 0.0;

  for (int i = 0; i < numberOfSegments ; ++i) {
      wp.x = wps[3 * i];
      wp.y = wps[3 * i + 1];
      wp.th = wps[3 * i + 2];
      path.push_back(wp);
  }

  WP_FOLLOW_PARAMS params = getRobotMotionParams();
  params.max_trans_vel = 0.5;

  XYTH_COORD currentHumPose;
  XYTH_COORD predHumPose;

  hist.addHeadIsValid(2.1, 4.1, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.5, 3.1, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.3, 2.11, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.4, 1.3, 0, 0, 0);
  usleep(1000000);
  hist.addHeadIsValid(2.5, 0.4, 0, 0, 0);


  VELOCITY vel = hist.bodyVelocity();
  currentHumPose.x = 2.5;
  currentHumPose.y = 0.12;
  currentHumPose.th = vel.heading_az;

  checkPosesInConflict(&currentHumPose, &vel, 1, currentRobotPose, robotVel, &path, 5, 0.8, params);
  return res;
}

int testPoseHistory()
{
  int res = 0;
  PoseHistory hist;
  XYTH_COORD currentHumPose;
  XYTH_COORD predHumPose;

  hist.addPoseIsValid(2.1, 4.1, 0, 0, 0);
  usleep(1000000);
  hist.addPoseIsValid(2.5, 3.1, 0, 0, 0);
  usleep(1000000);
  hist.addPoseIsValid(2.3, 2.11, 0, 0, 0);
  usleep(1000000);
  hist.addPoseIsValid(2.4, 1.3, 0, 0, 0);
  usleep(1000000);
  hist.addPoseIsValid(2.5, 0.4, 0, 0, 0);


  VELOCITY vel = hist.velocity();
  currentHumPose.x = 2.5;
  currentHumPose.y = 0.12;
  currentHumPose.th = vel.heading_az;


  printf("Velocity: %f x:%f y:%f heading:%f\n", vel.speedms, vel.speedxms, vel.speedyms, vel.heading_az);
  return res;
}

}

int main() {
  int tests = 0;
  int result = 0;

  NHPPlayerDriver::XYTH_COORD wp;
  std::list<NHPPlayerDriver::XYTH_COORD> path;

  // TODO: add asserts in tests

  result +=NHPPlayerDriver::testLinearprojection() + 1;
  tests += 1;
  printf("Test: %d Result: %d \n", tests, (tests-result));

  result +=NHPPlayerDriver::testPathProjection() + 1;
  tests += 1;
  printf("Test: %d Result: %d \n", tests, (tests-result));


  result +=NHPPlayerDriver::testPoseHistory() + 1;
  tests += 1;
  printf("Test: %d Result: %d \n", tests, (tests-result));

  result +=NHPPlayerDriver::testPoseConflict() + 1;
    tests += 1;
    printf("Test: %d Result: %d \n", tests, (tests-result));


  if (tests == result) {
      printf("Passed\n");
  } else {
       printf("Error: %d\n", result);
  }
}
