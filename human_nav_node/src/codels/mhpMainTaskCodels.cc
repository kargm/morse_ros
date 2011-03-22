#include "mhpm3d.h"
#include <display.h>
#include <mhpError.h>
#include <globalVars.h>

ACTIVITY_EVENT mhpLoadP3dMain(MHP_P3D *P3d, int *report)
{
  int col_mode_to_be_set = p3d_col_mode_pqp; 
  int FILTER_TO_BE_SET_ACTIVE = FALSE ;
  char *argv[2];
  int argc;
  p3d_env * env;
  int i;

  printf("%s ......loading\n", P3d->P3dModeleName.name);
  
  argv[0] = (char*) "MHP";
  argv[1] = P3d->P3dModeleName.name;
  argc = 2; 
  
  p3d_init_random(); 
  set_collision_by_object(FALSE);
 
  //p3d_BB_set_selection_method(DEACTIVATE_BB); 
  p3d_filter_switch_filter_mechanism(FILTER_TO_BE_SET_ACTIVE);
  
  if(P3d->enableGraphic){
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
    if ( p3d_read_desc((char *) P3d->P3dModeleName.name) == FALSE){
      *report =  S_mhp_FILE_NOT_FOUND;
      printf("Bad file type or file does not exist\n");
      return ETHER;
    }
    
    if(p3d_get_desc_number(P3D_ENV)) 
      printf("\n p3d loading done...\n"); 
    else{
      printf("\n error loading p3d...\n");
      *report = S_mhp_NOT_INITIALIZED;
      return ETHER;
    }
  }
  
  p3d_col_set_mode(col_mode_to_be_set);

  p3d_col_start(col_mode_to_be_set);  

  p3d_set_env_dmax(0.02); 

  if(P3d->enableGraphic){

    mhp_initialize_interface();
  }
//
//  /* --------- Internal data and Poster Initialization -------- */
//
//  SDI_F->NavParams.path_segment_length = MHP_NAV_DEFAULT_PATH_SEG_LENGTH;
//  SDI_F->NavParams.grid_sampling       = MHP_NAV_DEFAULT_GRID_SAMPLING;
//  SDI_F->NavParams.follow_distance     = MHP_NAV_DEFAULT_FOLLOW_DIST;
//
//  SDI_F->InterfaceParams.width = 800;
//  SDI_F->InterfaceParams.height = 600;
//  SDI_F->InterfaceParams.saveInterface            = GEN_FALSE;
//  SDI_F->InterfaceParams.show_nav_obstacles       = GEN_FALSE;
//  SDI_F->InterfaceParams.show_nav_distance_grid   = GEN_FALSE;
//  SDI_F->InterfaceParams.show_nav_visibility_grid = GEN_FALSE;
//  SDI_F->InterfaceParams.show_nav_hidzones_grid   = GEN_FALSE;
//  SDI_F->InterfaceParams.floor                    = GEN_TRUE;
//  SDI_F->InterfaceParams.walls                    = GEN_TRUE;
//  SDI_F->InterfaceParams.tiles                    = GEN_FALSE;
//  SDI_F->InterfaceParams.shadows                  = GEN_FALSE;
//
//  SDI_F->InterfaceState.savingInterface             = GEN_FALSE;
//  SDI_F->InterfaceState.updating                    = GEN_FALSE;
//  SDI_F->InterfaceState.showing_nav_obstacles       = GEN_FALSE;
//  SDI_F->InterfaceState.showing_nav_distance_grid   = GEN_FALSE;
//  SDI_F->InterfaceState.showing_nav_visibility_grid = GEN_FALSE;
//  SDI_F->InterfaceState.showing_nav_hidzones_grid   = GEN_FALSE;
//  SDI_F->InterfaceState.showing_floor               = GEN_TRUE;
//  SDI_F->InterfaceState.showing_walls               = GEN_TRUE;
//  SDI_F->InterfaceState.showing_tiles               = GEN_FALSE;
//  SDI_F->InterfaceState.showing_shadows             = GEN_FALSE;
//
//  SDI_F->VisParams.FoA_h = DTOR(MHP_DEFAULT_HUMAN_FOA_H);
//  SDI_F->VisParams.FoA_v = SDI_F->VisParams.FoA_h * 0.75;
//  SDI_F->VisParams.FoV_h = DTOR(MHP_DEFAULT_HUMAN_FOV_H);
//  SDI_F->VisParams.FoV_v = SDI_F->VisParams.FoV_h * 0.75;
//  SDI_F->VisParams.Vis_Ratio = MHP_DEFAULT_VISIBILITY_RATIO;
//
//  SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//  SDI_F->collisionState.mode  = GENMANIP_ON;
  

  /* -------------------------------------------------------------------- */
  
  assignGlobalVariables();
  
  if (MHP_ROBOT == NULL) {
    printf("**MHP** P3D did not contain a robot (Missing Macro files?).\n");
    *report = S_mhp_ERROR_UNKNOWN;
    return FAIL;
  }
  
  if(FAIL == mhp_initialize_navigation(report)){
    return FAIL;
  }   
  
//  if(FAIL == mhp_initialize_manipulation(report)){
//    return FAIL;
//  }

//#if defined(LIGHT_PLANNER)
//  for(int i=0; i<XYZ_ENV->nr; i++) {
//    deactivateCcCntrts(XYZ_ENV->robot[i], -1);
//  }
//#endif
  
  p3d_init_iksol(MHP_ROBOT->cntrt_manager);
  
  /* Initialize Agents structure */
  MHP_AGENTS = hri_create_agents();
  hri_assign_global_agents(MHP_AGENTS);
#ifdef JIDO
  hri_assign_source_agent("JIDOKUKA_ROBOT", MHP_AGENTS);
#elif HRP2
  hri_assign_source_agent("HRP2_ROBOT", MHP_AGENTS);
#else
  if (MHP_AGENTS->robots != NULL) {
    hri_assign_source_agent(MHP_AGENTS->robots[0]->robotPt->name, MHP_AGENTS);
  } else {
    hri_assign_source_agent("ROBOT", MHP_AGENTS);
  }
#endif
  /* End Initialize Agents */

  // Set the robots to initial Pos if defined in p3d file
  for(i = 0; i<MHP_ENV->nr; i++){
    if(!p3d_isNullConfig(MHP_ENV->robot[i], MHP_ENV->robot[i]->ROBOT_POS)){
      p3d_set_and_update_this_robot_conf(MHP_ENV->robot[i], MHP_ENV->robot[i]->ROBOT_POS);
    }
  }

#if defined (HRI_COSTSPACE)
  Graphic::initDrawFunctions();
  global_ActiveRobotName = MHP_ROBOT->name;
  global_Project = new Project(new Scene(XYZ_ENV));
  HRICS_init();
  //ENV.setBool(Env::drawGrid,true);
#endif

  printf ("\n****************** MHP INITIALIZED *********************\n");
  

  *report = OK;
  return ETHER;
}

//
///* mhpLoadSceMain  -  codel EXEC of LoadSce
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT mhpLoadSceMain(MHP_SCE *Sce, int *report)
//{
//  int i;
//
//  if(MHP_ENV == NULL){
//    printf("**MHP** MHP is not properly initialized.\n");
//    *report = S_mhp_NOT_INITIALIZED;
//    return ETHER;
//  }
//
//  p3d_read_scenario((char *) Sce->SceFileName.name);
//
//  for(i=0; i<MHP_ENV->nr; i++){
//    p3d_set_and_update_this_robot_conf(MHP_ENV->robot[i],MHP_ENV->robot[i]->ROBOT_POS);
//  }
//
//  return ETHER;
//}
//
//
///* Function assigning move3d robots to glocal variables */
//static int assignGlobalVariables()
//{
//  p3d_env * env;
//  int i;
//
//  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
//  for(i=0; i<env->nr; i++){
//    if( strcasestr(env->robot[i]->name,"robot") ){
//      MHP_ROBOT = env->robot[i];
//      continue;
//    }
//    if( strcasestr(env->robot[i]->name,"visball") ){
//      MHP_VISBALL = env->robot[i];
//      continue;
//    }
//    if( !strcmp("track",env->robot[i]->name) ){
//      trackDisc = env->robot[i];
//      continue;
//    }
//    if( !strcmp("pspHuman",env->robot[i]->name) ){
//      pspHuman = env->robot[i];
//      continue;
//    }
//  }
//
//  MHP_ENV = env;
//
//  return TRUE;
//}
//
//
///* mhpSetEnvironmentPosterMain  -  codel EXEC of SetEnvironmentPoster
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpSetEnvironmentPosterMain(GEN_STRING64 *EnvPoster, int *report)
//{
//  if(posterFind(EnvPoster->name, &posterEnvID)==ERROR) {
//    *report = S_mhp_POSTER_NOT_FOUND;
//    printf("Can't set Environment Poster\n");
//    SDI_F->State.envPosterAv = FALSE;
//  }
//  else {
//    SDI_F->State.envPosterAv = TRUE;
//  }
//
//  return ETHER;
//}
//
//
//int mhp_fetch_and_update_env(int *report)
//{
//#if defined(SPARK)
//  SPARK_CURRENT_ENVIRONMENT envPoster;
//  int i,j;
//  p3d_rob *robotPt;
//
//  if(SDI_F->State.envPosterAv) {
//    if(posterRead(posterEnvID,0, &envPoster, sizeof(envPoster)) != sizeof(envPoster)) {
//      *report = S_mhp_POSTER_NOT_READ;
//      return FALSE;
//    }
//
//    if( strcmp(envPoster.envName.name, MHP_ENV->name) || (envPoster.robotNb!=MHP_ENV->nr)) {
//      *report = S_mhp_POSTER_NOT_COMPATIBLE;
//      return FALSE;
//    }
//    else {
//      for(i=0; i<envPoster.robotNb; i++) {
//	robotPt = MHP_ENV->robot[i];
//
//	if(strcmp(envPoster.robot[i].name.name, robotPt->name) || (envPoster.robot[i].length!=robotPt->nb_dof)) {
//	  *report = S_mhp_POSTER_NOT_COMPATIBLE;
//	  return FALSE;
//	}
//	else {
//	  for(j=0; j<robotPt->nb_dof; j++) {
//	    robotPt->ROBOT_POS[j] = envPoster.robot[i].q[j];
//	  }
//	  p3d_set_and_update_this_robot_conf(robotPt, robotPt->ROBOT_POS);
//	}
//      }
//    }
//    g3d_draw_allwin_active();
//    return TRUE;
//  }
//  else {
//    return FALSE;
//  }
//
//#endif
//}
//
///* mhpFetchEnvironmentMain  -  codel EXEC of FetchEnvironment
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpFetchEnvironmentMain(int *report)
//{
//  if(!mhp_fetch_and_update_env(report))
//    printf("Could not fetch the environment");
//
//  return ETHER;
//}
//
///*------------------------------------------------------------
// *  Execution Tasks for a GLUT display
// *------------------------------------------------------------*/
//#ifdef USE_GLUT
//STATUS mhpMainTaskStart(int *report){
//  return OK;
//}
//
//STATUS
//mhpMainTaskEnd(){
//  return OK;
//}
//
//STATUS
//mhpMainTaskPerm(int *report){
//  if(initIsDone == false) {
//    return OK;
//  }
//  if(SDI_F->P3d.enableGraphic==GEN_TRUE) {
//    interface_MainLoopEvent();
//  }
//  return OK;
//}
//#endif


