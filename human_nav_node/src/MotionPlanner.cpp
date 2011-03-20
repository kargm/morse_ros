/*
 * MotionPlanner.cpp
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#include "human_nav_node/MotionPlanner.h"

#include <iostream>
#include <mhpm3d.h>

using namespace std;

p3d_env * MHP_ENV = NULL;
p3d_rob * MHP_ROBOT = NULL;

MotionPlanner::MotionPlanner() {
  isInitialized = false;
}

MotionPlanner::~MotionPlanner() {
}

int MotionPlanner::init(string filename, bool showInterface) {
  p3d_env * env;
  cout<<"Loading file: "<<filename<<"\n";

  isInitialized = true;
  return 0;
}
