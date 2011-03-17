/*
 * testxmlrpc-client.c
 *
 *  Created on: Mar 17, 2011
 *      Author: kruset
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <human_aware_navigation/Move3dXmlRpcClient.hpp>
#include <math.h>

using namespace human_aware_navigation;

int main(int argc, char* argv[])
{
//  if (argc != 4) {
//    printf("Usage: FileClient serverHost serverPort requestXmlFile\n");
//    return -1;
//  }
  int port = 7011; // 7011 standard port
  const char* host = "localhost";

  Moved3dXmlRpcClient* c =  new Moved3dXmlRpcClient(host, port);

  AbsoluteTrajectory waypoints; // result
  Human humans[MHPD_MAX_HUMANS];

  Human h1;
  h1.x = 5.5;
  h1.y = 5.5;
  h1.az = 0.0;
  h1.pose = STANDING_TRANSPARENT;
  h1.locked = 0;
  h1.exists = 1;
  h1.lastMoveTimeStampSecs = 0;
  humans[0] = h1;

  int human_no = 1;

  c->planPath(5.0, 5.0, 0.0, // pos
                      6.0, 6.0, 0.0, // goal
                      humans, human_no, // humans
                      &waypoints,
                      0);

  std::cout << "Got path with costs " << waypoints.costs;

  return 0;
}

