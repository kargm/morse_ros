/*
 * MotionPlanner.cpp
 *
 * The main class of the MHP Wrapper, containing all global vars,
 * calling codel functions copied from MHP.
 *
 * Ideally, this class does not expose mhp/genom/h2/genBasic/etc. API
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#include "human_nav_node/MotionPlanner.h"

#include <iostream>
#include <mhpm3d.h>
#include <mhpError.h>
#include <globalVars.h>
#include <display.h>


using namespace std;


#ifdef QT_LIBRARY
#include "Qt-pkg.h"
#include <QtGui/QIcon>
#include <QtGui/QPixmap>
QApplication* mhpInterfaceApp = NULL;
GLWidget* mhpInterfaceOpenGlWidget = NULL;
#endif

#ifdef USE_GLUT
#include <GL/freeglut.h>
GlutWindowDisplay* glutWin;
#endif

p3d_env * MHP_ENV = NULL;
p3d_rob * MHP_ROBOT = NULL;
HRI_AGENTS * MHP_AGENTS = NULL;
p3d_rob * MHP_VISBALL = NULL;
p3d_rob * trackDisc = NULL;
p3d_rob * pspHuman = NULL;

hri_gik * MHP_GIK = NULL;
//gik_task_joints * GIK_TASKS = NULL;
hri_bitmapset * MHP_BTSET = NULL;
MHP_INTERFACE_STATE InterfaceState;


double grid_sampling = 0.15;
int enableGraphic;


/**
 *
 */
void MotionPlanner::updateInterface() {
	if (isInitialized && enableGraphic) {
		int report;
		mhpUpdateInterfaceMain(&report);
		interface_MainLoopEvent();
	}
}

/**
 * loads p3d file
 */
int MotionPlanner::init(string filename, bool initShowInterface) {
//	char *filename_char = new char[filename.size()+1];
//	strcpy(filename_char, filename.c_str());

	if (isInitialized) {
		return OK;
	}

	enableGraphic = initShowInterface;
	MHP_P3D P3dspec;
	strncpy(P3dspec.P3dModeleName.name, filename.c_str(), 128);

	P3dspec.enableGraphic = enableGraphic;
	P3dspec.enablePersp = 0;
	int report = 0;
	mhpLoadP3dMain(&P3dspec, &report);
	if (report == OK) {
		isInitialized=true;
	}

	// TODO: remove
	MHP_CAM_POS pos;
	pos.dist = 4;
	pos.hrot = 0;
	pos.vrot = 0.2;
	pos.xdest = 2;
	pos.ydest = 0;
	pos.zdest = 2;
	changeCameraPosMain(&pos,&report);



	return report;
}

int MotionPlanner::initScene(string filename)
{
	int report = OK;
	MHP_SCE sce;
	strncpy(sce.SceFileName.name, filename.c_str(), 128);
	mhpLoadSceMain(&sce, &report);
	return report;
}

int MotionPlanner::changeCameraPosMain(MHP_CAM_POS *cam_pos, int *report)
{
  if(!enableGraphic){
    return OK;
  }

  mhpChangeCameraPosMain(cam_pos, report);
  return OK;
}

int MotionPlanner::initialize_navigation()
{
	int report;
	mhp_initialize_navigation(&report);
	return report;
}

int MotionPlanner::setInterfaceParams(MHP_INTERFACE_PARAMS *newParams, int *report)
{
	mhpSetInterfaceParamsMain(newParams, report);
	return *report;
}



int MotionPlanner::updPosAndFindNavTrajExec(MHP_UPD_FINDPATH &findpath_params, MHP_NAV_TRAJECTORY &result, int *report)
{
	*report = OK;
	if (! isInitialized) {
		return S_mhp_NAV_NOT_INITIALIZED;
	}
	mhpUpdPosAndFindNavTrajExec(findpath_params, result, report);
	return *report;
}

int MotionPlanner::findNavTrajExec(MHP_NAV_POS &MotionCoord, MHP_NAV_TRAJECTORY &result, int *report)
{
	if (! isInitialized) {
		return S_mhp_NAV_NOT_INITIALIZED;
	}

	mhpFindNavTrajExec(MotionCoord, result, report);


  return *report;
}

/* mhpPlaceAgentMain  -  codel EXEC of PlaceAgent
   Returns:  EXEC END ETHER FAIL ZOMBIE */
int MotionPlanner::placeAgent(MHP_AGENT_POSITION *addedAgent, int *report)
{
	// TODO
	if (! isInitialized) {
		return S_mhp_NAV_NOT_INITIALIZED;
	}
	mhpPlaceAgentMain(addedAgent, report);

  return OK;
}

