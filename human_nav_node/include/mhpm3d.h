/*****************************************************************************
*
* Emrah Akin Sisbot
* November 06 
*
* File : mhpm3d.h
* Brief   :
*
******************************************************************************/

#ifndef MHPM3D_H
#define MHPM3D_H

/*----------------------------------------------------------------------------
  INCLUDE
-----------------------------------------------------------------------------*/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "Move3d-pkg.h"
#include "../hri_planner/proto/hri_gik_proto.h"
#include "../hri_planner/proto/hri_bitmap_proto.h"
// for module
#include "mhpStruct.h"

//#include "../lightPlanner/proto/lightPlannerApi.h"
//#include "../lightPlanner/proto/lightPlanner.h"

//#ifdef JIDO
//#include "GraspPlanning-pkg.h"
//#include "../lightPlanner/proto/Manipulation.h"
//#include "../lightPlanner/proto/ManipulationPlanner.hpp"
//#include "../lightPlanner/proto/ManipulationUtils.hpp"
//#include "../p3d/env.hpp"
//#endif

//#endif

#define MHP_D2D(x1,y1,x2,y2) (sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)))
#define MHP_D3D(x1,y1,z1,x2,y2,z2) (sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1)))

#define COND_PRINT(cond,str)  if( cond ) printf str
#define DEBUG_PRINTF(level,str)  if(SDI_F->State.debug_mode && (level <= SDI_F->State.debug_mode) ) printf str

extern hri_bitmapset * BTSET; // BTSET is a global variable used in the visualization of Move3D
//extern hri_bitmapset * ACBTSET; // ACBTSET is a global variable used for mightabilities
// TODO: Change mightabilities to accpt BTSET as a parameter.
//extern FL_FORM *MAIN_FORM;

extern p3d_env * MHP_ENV;
extern p3d_rob * MHP_ROBOT;
extern hri_bitmapset * MHP_BTSET;
//extern hri_gik * MHP_GIK;
//extern HRI_AGENTS * MHP_AGENTS;
//extern int Mightability_Maps_Initialized;


/*----------------------------------------------------------------------------
  STRUCTURES
  -----------------------------------------------------------------------------*/

//typedef struct {
//  int default_joints[MHP_GIK_MAX_JOINT_NO];
//  int default_joints_no;
//  int actual_joints[MHP_GIK_MAX_JOINT_NO];
//  int actual_joints_no;
//  int active_joint;
//} gik_task_joints;

/*----------------------------------------------------------------------------
  PROTOTYPE
-----------------------------------------------------------------------------*/

extern int mhp_fetch_and_update_env(int *report);

//extern gik_task_joints * mhp_create_defaut_gik_tasks( void );
//extern ACTIVITY_EVENT mhp_initialize_manipulation(int *report);
//extern ACTIVITY_EVENT mhp_initialize_navigation(int *report);
extern void mhp_initialize_interface( void ); 
#if defined(JIDO)
extern void mhp_Arm2ConfigPt(Gb_q7 *gbconfig, configPt m3dconfig);
#endif
#ifdef USE_GLUT
extern void interface_MainLoopEvent();
#endif
#endif
