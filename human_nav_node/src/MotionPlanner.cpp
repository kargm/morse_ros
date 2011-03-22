/*
 * MotionPlanner.cpp
 *
 * The main class of the MHP Wrapper, containing all global vars,
 * calling codel functions copied from MHP.
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
hri_bitmapset * MHP_BTSET = NULL;

double grid_sampling = 0.15;


/* Function assigning move3d robots to global variables */
int assignGlobalVariables()
{
  p3d_env * env;
  int i;

  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  for(i=0; i<env->nr; i++){
    if( strcasestr(env->robot[i]->name,"robot") ){
      MHP_ROBOT = env->robot[i];
      continue;
    }
    if( strcasestr(env->robot[i]->name,"visball") ){
      MHP_VISBALL = env->robot[i];
      continue;
    }
    if( !strcmp("track",env->robot[i]->name) ){
      trackDisc = env->robot[i];
      continue;
    }
    if( !strcmp("pspHuman",env->robot[i]->name) ){
      pspHuman = env->robot[i];
      continue;
    }
  }

  MHP_ENV = env;

  return TRUE;
}

void mhp_draw_allwin_active()
{
#ifdef USE_GLUT
  g3d_glut_paintGL();
  glutPostRedisplay ();
#endif
#ifdef QT_LIBRARY
  mhpInterfaceOpenGlWidget->updateGL();
#endif
}

void mhp_initialize_interface()
{
  G3D_Window *window;

#ifdef QT_LIBRARY

  mhpInterfaceOpenGlWidget = new GLWidget(NULL);
  mhpInterfaceOpenGlWidget->setObjectName(QString::fromUtf8("Mhp OpenGL"));
  mhpInterfaceOpenGlWidget->show();
  mhpInterfaceOpenGlWidget->raise();
#endif
#ifdef USE_GLUT
    glutWin->initDisplay();
#endif

  window = g3d_get_win_by_name((char*)"Move3D");
  g3d_set_win_floor_color(window->vs, 0.39, 0.68, 0.84);
  window->vs.displayFloor = TRUE;
  window->vs.displayWalls = TRUE;
  ext_g3d_draw_allwin_active = (void (*)())(mhp_draw_allwin_active);
}

void interface_MainLoopEvent(){
#ifdef USE_GLUT
		glutMainLoopEvent ();
#endif
}

/**
 *
 */
void MotionPlanner::updateInterface() {
	if (isInitialized && showInterface) {
		interface_MainLoopEvent();
	}

}

/**
 * loads p3d file
 */
int MotionPlanner::init(string filename, bool initShowInterface) {
//	char *filename_char = new char[filename.size()+1];
//	strcpy(filename_char, filename.c_str());

	showInterface = initShowInterface;
	MHP_P3D P3dspec;
	strncpy(P3dspec.P3dModeleName.name, filename.c_str(), 128);

	P3dspec.enableGraphic = showInterface;
	P3dspec.enablePersp = 0;
	int report;
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


int MotionPlanner::changeCameraPosMain(MHP_CAM_POS *cam_pos, int *report)
{
  G3D_Window *window = g3d_get_win_by_name((char*)"Move3D");
  *report = OK;
  if(!showInterface){
    printf("The Interface is disabled. Cannot change the camera position.\n");
    *report = S_mhp_NO_INTERFACE_LAUCHED;
    return OK;
  }

  else{
    if(window){
      printf("Changing camera position of the window\n");
      g3d_set_win_camera(window->vs, cam_pos->xdest, cam_pos->ydest, cam_pos->zdest,
			 cam_pos->dist, cam_pos->hrot, cam_pos->vrot, 0.0, 0.0, 1.0);
    }
  }
  return OK;
}

int MotionPlanner::initialize_navigation()
{
  int dimx,dimy;

  MHP_BTSET = hri_bt_create_bitmaps();

  if(MHP_BTSET==NULL){
    printf("Problem in navigation initialization.\n");
    return ERROR;
  }
  else{
    printf("MHP is set with %d humans\n",MHP_BTSET->human_no);
  }

  if(MHP_ENV==NULL){
    printf("Problem in navigation initialization. Environment is NULL\n");
    return ERROR;
  }

  dimx = (int)((MHP_ENV->box.x2 - MHP_ENV->box.x1)/grid_sampling);
  dimy = (int)((MHP_ENV->box.y2 - MHP_ENV->box.y1)/grid_sampling);

  if(!hri_bt_init_bitmaps(MHP_BTSET, dimx, dimy, 1, grid_sampling)){
    printf("**MHP** Can't initialize grids\n");
    /* TODO : free the BTSET */
    return ERROR;
  }
  else{
    printf("**MHP** Navigation Grids succesfully initialized\n");
  }

  BTSET = MHP_BTSET; // BTSET is a global variable used in the visualization of Move3D

  return OK;
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

	// TODO
	*report = OK;

  return *report;
}

/* mhpPlaceAgentMain  -  codel EXEC of PlaceAgent
   Returns:  EXEC END ETHER FAIL ZOMBIE */
int MotionPlanner::mhpPlaceAgentMain(MHP_AGENT_POSITION *addedAgent, int *report)
{
	// TODO
	if (! isInitialized) {
		return S_mhp_NAV_NOT_INITIALIZED;
	}
  return OK;
}

