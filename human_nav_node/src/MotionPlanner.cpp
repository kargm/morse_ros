/*
 * MotionPlanner.cpp
 *
 *  Created on: Mar 20, 2011
 *      Author: kruset
 */

#include "human_nav_node/MotionPlanner.h"

#include <iostream>
#include <mhpm3d.h>
#include <mhpError.h>


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

using namespace std;

p3d_env * MHP_ENV = NULL;
p3d_rob * MHP_ROBOT = NULL;
HRI_AGENTS * MHP_AGENTS = NULL;
static p3d_rob * MHP_VISBALL = NULL;
static p3d_rob * trackDisc = NULL;
static p3d_rob * pspHuman = NULL;


/* Function assigning move3d robots to glocal variables */
static int assignGlobalVariables()
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

int MotionPlanner::init(string filename, bool showInterface) {
	int col_mode_to_be_set = p3d_col_mode_pqp;
	int FILTER_TO_BE_SET_ACTIVE = FALSE ;
	char *argv[2];
	int argc;
	char *filename_char = new char[filename.size()+1];
	strcpy(filename_char, filename.c_str());


	cout<<"Loading file: "<<filename<<"\n";

	p3d_init_random();
	set_collision_by_object(FALSE);

	//p3d_BB_set_selection_method(DEACTIVATE_BB);
	p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);

	argv[0] = (char*) "MHP";
	argv[1] = filename_char;
	argc = 2;

	if(showInterface){
#ifdef USE_GLUT
		glutWin = new GlutWindowDisplay(argc,argv);
#else
#ifdef QT_LIBRARY
		mhpInterfaceApp = new QApplication(argc,argv );
#endif
#endif
	}

	while(!p3d_get_desc_number(P3D_ENV)) {
		p3d_col_set_mode(p3d_col_mode_none);
		p3d_BB_set_mode_close();
		if ( p3d_read_desc(filename_char) == FALSE){
			cerr<<"Bad file type or file does not exist: "<<filename.c_str()<<"\n";
			return S_mhp_FILE_NOT_FOUND;
		}

		if(p3d_get_desc_number(P3D_ENV))
			printf("\n p3d loading done...\n");
		else{
			printf("\n error loading p3d...\n");
			return S_mhp_NOT_INITIALIZED;
		}
	}

	p3d_col_set_mode(col_mode_to_be_set);

	p3d_col_start(col_mode_to_be_set);

	p3d_set_env_dmax(0.02);

	if(showInterface){

		mhp_initialize_interface();
	}

	 assignGlobalVariables();

	  if (MHP_ROBOT == NULL) {
	    printf("**MHP** P3D did not contain a robot (Missing Macro files?).\n");
	    return S_mhp_ERROR_UNKNOWN;
	  }


	p3d_init_iksol(MHP_ROBOT->cntrt_manager);

	/* Initialize Agents structure */
	MHP_AGENTS = hri_create_agents();
	hri_assign_global_agents(MHP_AGENTS);

	// Set the robots to initial Pos if defined in p3d file
	  for(int i = 0; i<MHP_ENV->nr; i++){
	    if(!p3d_isNullConfig(MHP_ENV->robot[i], MHP_ENV->robot[i]->ROBOT_POS)){
	      p3d_set_and_update_this_robot_conf(MHP_ENV->robot[i], MHP_ENV->robot[i]->ROBOT_POS);
	    }
	  }

	  printf ("\n****************** MHP INITIALIZED *********************\n");

	delete filename_char;
	isInitialized = true;
	return 0;
}
