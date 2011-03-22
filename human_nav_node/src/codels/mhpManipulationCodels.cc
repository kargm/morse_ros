#include "mhpm3d.h"
#include <display.h>
#include <mhpError.h>
#include <globalVars.h>



//static POSTER_ID posterArmPoseOnTraj = NULL;


#if defined(JIDO)
static ManipulationPlanner * MANIP_JIDO = NULL;
#endif


ACTIVITY_EVENT mhp_initialize_manipulation(int *report)
{
  
  MHP_GIK = hri_gik_create_gik();
  
  if(MHP_GIK == NULL){
    printf("**MHP** Problem occured in the creation of GIK\n");
    *report =  S_mhp_GIK_NOT_CREATED;
    return FAIL;
  }
  
//  GIK_TASKS = mhp_create_defaut_gik_tasks();
//  if(GIK_TASKS == NULL){
//    printf("**MHP** Problem occured in the creation of default motions. Wrong robot choice?\n");
//    *report =  S_mhp_GIK_NOT_CREATED;
//    return FAIL;
//  }
  
  if (MHP_BTSET !=NULL && MHP_BTSET->human !=NULL && MHP_BTSET->robot != NULL){
    MHP_GIK->robot = MHP_BTSET->robot;
    MHP_GIK->human = MHP_BTSET->human;
    MHP_GIK->human_no = MHP_BTSET->human_no;
  }

#if defined(JIDO)

  if (MANIP_JIDO == NULL) {
    p3d_rob * robotPt= MHP_ROBOT;
    if(robotPt==NULL)
      printf("cannot fetch the robot by its name\n");
    else {
      printf("initialize manip_jido\n");
      MANIP_JIDO= new ManipulationPlanner(robotPt);
      printf("initialize manip_jido   OK\n");
    }
  }
#endif
  return ETHER;
}

///* mhpDoTaskWithAgentMain  -  codel EXEC of DoTaskWithAgent
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpDoTaskWithAgentMain(MHP_AGENT_MOTION_TASK *motionTask, MHP_AGENT_MOTION_TASK_RESULT *motionTaskResult, int *report)
//{
//  int i,j;
//  HRI_AGENT *agent = NULL;
//  p3d_rob *object = NULL;
//  configPt q, qs;
//  p3d_vector3 Tcoord[3];
//
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  for(i=0; i<MHP_AGENTS->all_agents_no; i++) {
//    if( strcasestr(MHP_AGENTS->all_agents[i]->robotPt->name, motionTask->name.name) ) {
//      agent = MHP_AGENTS->all_agents[i];
//      break;
//    }
//  }
//  if(agent == NULL){
//    *report = S_mhp_AGENT_NOT_FOUND;
//    return ETHER;
//  }
//  for(i=0; i<MHP_ENV->nr; i++){
//    if( strcasestr(MHP_ENV->robot[i]->name, motionTask->task.target.name) ){
//      object = MHP_ENV->robot[i];
//      break;
//    }
//  }
//  if(object == NULL){
//    *report = S_mhp_OBJECT_UNKNOWN;
//    return ETHER;
//  }
//
//  qs = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */
//  q  = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */
//
//  p3d_get_robot_config_into(agent->robotPt, &qs);
//
//  Tcoord[0][0] = Tcoord[1][0] = Tcoord[2][0] = object->joints[1]->abs_pos[0][3];
//  Tcoord[0][1] = Tcoord[1][1] = Tcoord[2][1] = object->joints[1]->abs_pos[1][3];
//  Tcoord[0][2] = Tcoord[1][2] = Tcoord[2][2] = object->joints[1]->abs_pos[2][3];
//
//  if(motionTask->task.motiontype == MHP_LOOK){
//    hri_agent_single_task_manip_move(agent, GIK_LOOK, object, &q);
//    p3d_set_and_update_this_robot_conf(agent->robotPt,q);
//    for(i=0, j=0; i<agent->robotPt->nb_dof && j<4; i++) {
//      if(q[i] != qs[i]) {
//	motionTaskResult->look[j] = RTOD(q[i]);
//	j++;
//      }
//    }
//    motionTaskResult->look_nb = j-1;
//  }
//  if(motionTask->task.motiontype ==  MHP_LEFT_HAND_POINT){
//    hri_agent_single_task_manip_move(agent, GIK_LAPOINT, object, &q);
//    p3d_set_and_update_this_robot_conf(agent->robotPt,q);
//    for(i=0, j=0; i<agent->robotPt->nb_dof && j<7; i++) {
//      if(q[i] != qs[i]) {
//	motionTaskResult->arm[j] = RTOD(q[i]);
//	j++;
//      }
//    }
//    motionTaskResult->look_nb = j-1;
//  }
//  if(motionTask->task.motiontype ==  MHP_RIGHT_HAND_POINT){
//    hri_agent_single_task_manip_move(agent, GIK_RAPOINT, object, &q);
//    p3d_set_and_update_this_robot_conf(agent->robotPt,q);
//    for(i=0, j=0; i<agent->robotPt->nb_dof && j<7; i++) {
//      if(q[i] != qs[i]) {
//	motionTaskResult->arm[j] = RTOD(q[i]);
//	j++;
//      }
//    }
//    motionTaskResult->look_nb = j-1;
//  }
//  if(motionTask->task.motiontype ==  MHP_LEFT_HAND_REACH){
//    hri_agent_single_task_manip_move(agent, GIK_LAREACH, object, &q);
//    p3d_set_and_update_this_robot_conf(agent->robotPt,q);
//    for(i=0, j=0; i<agent->robotPt->nb_dof && j<7; i++) {
//      if(q[i] != qs[i]) {
//	motionTaskResult->arm[j] = RTOD(q[i]);
//	j++;
//      }
//    }
//    motionTaskResult->look_nb = j-1;
//  }
//  if(motionTask->task.motiontype ==  MHP_RIGHT_HAND_REACH){
//    hri_agent_single_task_manip_move(agent, GIK_RAREACH, object, &q);
//    p3d_set_and_update_this_robot_conf(agent->robotPt,q);
//    for(i=0, j=0; i<agent->robotPt->nb_dof && j<7; i++) {
//      if(q[i] != qs[i]) {
//	motionTaskResult->arm[j] = RTOD(q[i]);
//	j++;
//      }
//    }
//    motionTaskResult->look_nb = j-1;
//  }
//
//  MY_FREE(q, double , agent->robotPt->nb_dof); /* FREE */
//  MY_FREE(qs, double, agent->robotPt->nb_dof); /* FREE */
//
//  return ETHER;
//}
//
//
///* mhpPlanPathStart  -  codel START of PlanPath
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpPlanPathStart(MHP_MOTION_TARGET *target, int *report)
//{
//  if(MHP_GIK == NULL){
//    printf("**MHP** GIK components are not created. Cannot plan a path\n");
//    *report =  S_mhp_NOT_INITIALIZED;
//    return ETHER;
//  }
//  else{
//    return EXEC;
//  }
//}
//
//
///* mhpPlanPathMain  -  codel EXEC of PlanPath
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpPlanPathMain(MHP_MOTION_TARGET *target, int *report)
//{
//  int i;
//  int Pathpointindex = 0;
//  double Tcoord[6];
//  gsl_vector * DT;
//  configPt robotq;
//  double begpos[6];
//  double curpos[6];
//  double  remainingdist = 100;
//  int max_count = 0;
//  double  maxdistance = 0;
//
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  if(!strcmp(target->target.name,"COORD")){
//    Tcoord[0] = target->coords.x; Tcoord[1] = target->coords.y;  Tcoord[2] = target->coords.z;
//    Tcoord[3] = target->coords.yaw; Tcoord[4] = target->coords.pitch; Tcoord[5] = target->coords.roll;
//  }
//  else{
//    if(MHP_ENV == NULL){
//      printf("**MHP** MHP not correctly initialized.\n ");
//      *report = S_mhp_NOT_INITIALIZED;
//      return FAIL;
//    }
//    for(i=0; i<MHP_ENV->nr; i++){
//      if( !strcmp(target->target.name,MHP_ENV->robot[i]->name) ){
//	p3d_get_robot_pos(MHP_ENV->robot[i],Tcoord);
//	break;
//      }
//    }
//    if(i==MHP_ENV->nr){
//      printf("Object not found. Cannot plan a path\n");
//      *report = S_mhp_OBJECT_UNKNOWN;
//      return FAIL;
//    }
//    else{
//      printf("Planning path to %s\n",MHP_ENV->robot[i]->name);
//    }
//  }
//  printf("MHP plans to reach the target: %f %f %f %f %f %f\n",Tcoord[0],Tcoord[1],Tcoord[2],Tcoord[3],Tcoord[4],Tcoord[5]);
//
//  /* ----Beginning GIK task----- */
//
//  /* TODO: It might be good to do simple distance tests to see if the robot can reach the goal */
//
//
//  /* initialize gik */
//
//  if( MHP_GIK == NULL || MHP_ROBOT == NULL){
//    printf("**MHP** MHP not correctly initialized.\n ");
//    *report = S_mhp_NOT_INITIALIZED;
//    return FAIL;
//  }
//  if(MHP_GIK->GIKInitialized){
//    hri_gik_destroy_gik_data(MHP_GIK);
//    hri_gik_uninitialize_gik(MHP_GIK);
//  }
//
//  DEBUG_PRINTF(3,("GIK Initialization\n"));
//  hri_gik_initialize_gik(MHP_GIK,MHP_ROBOT,GIK_TASKS[target->motiontype].actual_joints_no);
//
//  DEBUG_PRINTF(3,("GIK Add Task\n"));
//  hri_gik_add_task(MHP_GIK, 3, GIK_TASKS[target->motiontype].actual_joints_no, 1,
//		   GIK_TASKS[target->motiontype].actual_joints,GIK_TASKS[target->motiontype].active_joint);
//
//  /* -----BEGIN PATH PLANNING----- */
//  DEBUG_PRINTF(3,("GIK beginning\n"));
//
//
//  for(i=0; i<MHP_GIK->task_no; i++){
//    gsl_vector_set(MHP_GIK->task[i]->goal, 0, Tcoord[0]);
//    gsl_vector_set(MHP_GIK->task[i]->goal, 1, Tcoord[1]);
//    gsl_vector_set(MHP_GIK->task[i]->goal, 2, Tcoord[2]); /* We take into account only the position */
//  }
//
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//
//  robotq = MY_ALLOC(double,MHP_GIK->robot->nb_dof); /* ALLOC */
//  p3d_get_robot_config_into(MHP_GIK->robot,&robotq);
//
//  DT = gsl_vector_alloc(MHP_GIK->joint_no); /* ALLOC */
//
//  p3d_mat4ExtractPosReverseOrder(MHP_GIK->robot->joints[GIK_TASKS[target->motiontype].active_joint]->abs_pos, begpos, begpos+1, begpos+2,begpos+3, begpos+4, begpos+5);
//
//  do{
//
//    for(i=0; i<MHP_GIK->task_no; i++)
//      hri_gik_computeJacobian(MHP_GIK,i,0);
//
//    hri_gik_free_joints(MHP_GIK);
//
//    hri_gik_compute_core(MHP_GIK, DT);
//
//    hri_gik_updaterobot(MHP_GIK, DT);
//
//    /* here the robot is moved a little towards the goal */
//
//    p3d_mat4ExtractPosReverseOrder(MHP_GIK->robot->joints[GIK_TASKS[target->motiontype].active_joint]->abs_pos,
//				   curpos, curpos+1, curpos+2, curpos+3, curpos+4, curpos+5);
//
//    if(MHP_D3D(begpos[0],begpos[1],begpos[2],curpos[0],curpos[1],curpos[2]) > 0.05){
//      p3d_get_robot_config_into(MHP_GIK->robot,&robotq);
//
//      /* robotq contains robot's configuration to be written into the poster */
//
//#ifdef JIDO
////      for(i=0; i<6; i++)
////	SDI_F->Path.trajectory[i][Pathpointindex] = robotq[12+i];        //Jido specific
////
////      Pathpointindex++;
////      SDI_F->Path.nb_positions = Pathpointindex;
//#elif defined(HRP2)
//      
//      /* TODO: --- HRP2 functions ---- */
//#elif defined(BH2)
//#elif defined(SIMULATION)
//#else
//#error "no poster writing function has been declared for the current robot"
//#endif
//      p3d_mat4ExtractPosReverseOrder(MHP_GIK->robot->joints[GIK_TASKS[target->motiontype].active_joint]->abs_pos, begpos, begpos+1, begpos+2,begpos+3, begpos+4, begpos+5);
//    }
//
//    maxdistance = 0;
//    for(i=0; i<MHP_GIK->task_no; i++){
//      remainingdist = hri_gik_remainingdistance(MHP_GIK,i);
//      if(remainingdist > maxdistance)
//	maxdistance = remainingdist;
//      DEBUG_PRINTF(4,("Remaining distance for task %d: %f\n",i, remainingdist));
//    }
//
//    max_count++;
//
//  }while( (max_count < 100) &&
//	  (maxdistance > 0.05) );
//
//  gsl_vector_free(DT); /* FREE */
//  MY_FREE(robotq, double, MHP_GIK->robot->nb_dof); /* FREE */
//
//  if(maxdistance > 0.05){
//    printf("**MHP** Warning planning couldn't reach the goal. Poster not complete\n ");
//    *report = S_mhp_NOT_REACHED;
//  }
//
//#ifdef JIDO
//  //  mhpPathPathPosterWrite ( MHP_PATH_POSTER_ID, &SDI_F->Path );
//#endif
//
//  SDI_F->State.planning = FALSE;
//
//  /* ------ End of Planning ------ */
//
//  return ETHER;
//}
//
//
///* mhpPlanPathInter  -  codel INTER of PlanPath
//   Returns:  INTER ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpPlanPathInter(MHP_MOTION_TARGET *target, int *report)
//{
//  SDI_F->State.planning = FALSE;
//
//  return ETHER;
//}
//
//
//#ifdef JIDO
//////////////////////////////////////////////////////////////
/////                   XAV's REQUESTS                     ///
//////////////////////////////////////////////////////////////
//
//
///* mhpCleanRoadmapMain  -  codel EXEC of CleanRoadmap
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpCleanRoadmapMain(int *report)
//{
//  SDI_F->State.planning = TRUE;
//
// SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//
//
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return ETHER;
//  }
//  MANIP_JIDO->cleanRoadmap();
//  printf("clean roadmap ok\n");
//  MANIP_JIDO->cleanTraj();
//  printf("clean traj ok\n");
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
//
//
///*------------------------------------------------------------------------
// * ArmPlanTask
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// *              S_mhp_NO_TRAJ_FOUND
// *              S_mhp_INVALID_QSTART
// *              S_mhp_INVALID_QGOAL
// *              S_mhp_INVALID_TRAJ_ID
// *              S_mhp_INVALID_TASK
// *              S_mhp_UNKNOWN_OBJECT
// *              S_mhp_NO_GRASP
// *              S_mhp_NO_PLACE
// *              S_mhp_ERROR_UNKNOWN
// *              S_mhp_EQUAL_QSTART_QGOAL
// */
//
///* mhpArmPlanTaskStart  -  codel START of ArmPlanTask
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmPlanTaskStart(STRUCT_MHP_ARM_PLAN_PARAMS *armPlanTaskParams, int *report)
//{
//  configPt qstart= NULL, qgoal= NULL;
//
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//
//  SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return ETHER;
// }
//
//  switch (armPlanTaskParams->taskType) {
//  case MHP_ARM_FREE :
//  case MHP_ARM_PICK_GOTO :
//  case MHP_ARM_TAKE_TO_FREE :
//  case MHP_ARM_TAKE_TO_PLACE :
//  case MHP_ARM_PLACE_FROM_FREE :
//  case MHP_ARM_EXTRACT :
//    break;
//  default :
//    *report = S_mhp_INVALID_TASK;
//    return END;
//  }
//
//  if(armPlanTaskParams->cartesian == GEN_TRUE){
//    printf("cartesian mode\n");
//    MANIP_JIDO->setArmCartesian(0, true);
//  } else {
//    MANIP_JIDO->setArmCartesian(0, false);
//  }
//
//  // mhpInitRobotPoseForPlanning
//  XYZ_ENV->cur_robot= MHP_ROBOT;
//  //printf("your robot is %s\n",MHP_ROBOT->name);
//  qstart= p3d_alloc_config(MHP_ROBOT);
//  qgoal= p3d_alloc_config(MHP_ROBOT);
//
//  p3d_get_robot_config_into(MHP_ROBOT, &qstart);
//
//
//  p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qstart);
//
//
//  if(armPlanTaskParams->startIsCurrent == GEN_FALSE){
//
//    mhp_Arm2ConfigPt(&armPlanTaskParams->armStart, qstart);
//    p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qstart);
//    p3d_get_robot_config_into(MHP_ROBOT, &qstart);
//  }
//  p3d_copy_config_into(MHP_ROBOT, qstart, &qgoal);
//  //printf("conf start\n");
//  //print_config(MHP_ROBOT, qstart);
//
//  mhp_Arm2ConfigPt(&armPlanTaskParams->armGoto, qgoal);
//
//  p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qgoal);
//
//  p3d_copy_config_into(MHP_ROBOT, qstart, &MHP_ROBOT->ROBOT_POS);
//  p3d_copy_config_into(MHP_ROBOT, qgoal, &MHP_ROBOT->ROBOT_GOTO);
//
//  p3d_set_and_update_this_robot_conf(MHP_ROBOT, qstart);
//  p3d_destroy_config(MHP_ROBOT, qstart);
//  p3d_destroy_config(MHP_ROBOT, qgoal);
//
//  //printf("conf MHP_ROBOT->ROBOT_POS\n");
//  //print_config(MHP_ROBOT, MHP_ROBOT->ROBOT_POS);
//  //printf("conf MHP_ROBOT->ROBOT_GOTO\n");
//  //print_config(MHP_ROBOT, MHP_ROBOT->ROBOT_GOTO);
//  return EXEC;
//}
//
///* mhpArmPlanTaskMain  -  codel EXEC of ArmPlanTask
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmPlanTaskMain(STRUCT_MHP_ARM_PLAN_PARAMS *armPlanTaskParams, int *report)
//{
//  int result;
//  int i =0;
//  int indexTraj = armPlanTaskParams->trajId;
//  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
//  std::vector <SM_TRAJ> smTrajs;
//  std::vector<double> objStart,objGoto;
//
//  if(indexTraj < 0 || indexTraj > MHP_ARM_NB_MAX_TRAJ) {
//    *report = S_mhp_INVALID_TRAJ_ID;
//    return END;
//  }
//  if(armPlanTaskParams->cartesian == GEN_TRUE) {
//    printf("cartesian mode\n");
//    MANIP_JIDO->setArmCartesian(0, true);
//  } else {
//    MANIP_JIDO->setArmCartesian(0, false);
//  }
//  objStart.resize(6);
//  objGoto.resize(6);
//  for(int i = 0; i < 6; i++){
//    objStart[i] = objGoto[i] = P3D_HUGE;
//  }
//  //Traslations
//  if(armPlanTaskParams->useObjStart & 1){
//    objStart[0] = armPlanTaskParams->objStart[0];
//    objStart[1] = armPlanTaskParams->objStart[1];
//    objStart[2] = armPlanTaskParams->objStart[2];
//  }
//  if(armPlanTaskParams->useObjGoto & 1){
//    objGoto[0] = armPlanTaskParams->objGoto[0];
//    objGoto[1] = armPlanTaskParams->objGoto[1];
//    objGoto[2] = armPlanTaskParams->objGoto[2];
//  }
//
//  //Bit computation to get the rotations to sample.
//  //When the Bit = 1 the corresponding rotation has to be sampled
//
//  if(armPlanTaskParams->useObjStart & 2){
//    objStart[3] = armPlanTaskParams->objStart[3];
//  }
//  if(armPlanTaskParams->useObjStart & 4){
//    objStart[4] = armPlanTaskParams->objStart[4];
//  }
//  if(armPlanTaskParams->useObjStart & 8){
//    objStart[5] = armPlanTaskParams->objStart[5];
//  }
//
//  if(armPlanTaskParams->useObjGoto & 2){
//    objGoto[3] = armPlanTaskParams->objGoto[3];
//  }
//  if(armPlanTaskParams->useObjGoto & 4){
//    objGoto[4] = armPlanTaskParams->objGoto[4];
//  }
//  if(armPlanTaskParams->useObjGoto & 8){
//    objGoto[5] = armPlanTaskParams->objGoto[5];
//  }
//
//  result = MANIP_JIDO->armPlanTask(MANIPULATION_TASK_TYPE_STR(armPlanTaskParams->taskType), 0,
//				   MANIP_JIDO->robotStart(),MANIP_JIDO->robotGoto(),objStart, objGoto, (char*)armPlanTaskParams->objectName.name, (char*)armPlanTaskParams->supportName.name, confs, smTrajs);
//  switch(result){
//    case MANIPULATION_TASK_OK:{
//      break;
//    }
//    case MANIPULATION_TASK_NOT_INITIALIZED:{
//      *report= S_mhp_NOT_INITIALIZED;
//      return END;
//    }
//    case MANIPULATION_TASK_NO_TRAJ_FOUND:{
//      *report= S_mhp_NO_TRAJ_FOUND;
//      return END;
//    }
//    case MANIPULATION_TASK_INVALID_QSTART:{
//      *report= S_mhp_INVALID_QSTART;
//      return END;
//    }
//    case MANIPULATION_TASK_INVALID_QGOAL:{
//      *report= S_mhp_INVALID_QGOAL;
//      return END;
//    }
//    case MANIPULATION_TASK_INVALID_TRAJ_ID:{
//      *report= S_mhp_INVALID_TRAJ_ID;
//      return END;
//    }
//    case MANIPULATION_TASK_INVALID_TASK:{
//      *report= S_mhp_INVALID_TASK;
//      return END;
//    }
//    case MANIPULATION_TASK_UNKNOWN_OBJECT:{
//      *report= S_mhp_UNKNOWN_OBJECT;
//      return END;
//    }
//    case MANIPULATION_TASK_NO_GRASP:{
//      *report= S_mhp_NO_GRASP;
//      return END;
//    }
//    case MANIPULATION_TASK_NO_PLACE:{
//      *report= S_mhp_NO_PLACE;
//      return END;
//    }
//    case MANIPULATION_TASK_ERROR_UNKNOWN:{
//      *report= S_mhp_ERROR_UNKNOWN;
//      return END;
//    }
//    case MANIPULATION_TASK_EQUAL_QSTART_QGOAL:{
//      *report= S_mhp_EQUAL_QSTART_QGOAL;
//      return END;
//    }
//  }
//
//
//  smTrajs[0].convertToSM_TRAJ_STR(&(SDI_F->armTrajArray.traj[indexTraj]));
//
//  printf("smTraj.nbAxis %d\n", SDI_F->armTrajArray.traj[indexTraj].nbAxis);
//
//  mhpArmTrajarmTrajectoryOuputPosterWrite( MHP_ARMTRAJ_POSTER_ID, &(SDI_F->armTrajArray.traj[indexTraj]));
//  return END;
//}
//
//
//
///* mhpArmPlanTaskEnd  -  codel END of ArmPlanTask
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmPlanTaskEnd(STRUCT_MHP_ARM_PLAN_PARAMS *armPlanTaskParams, int *report)
//{
//  deactivateCcCntrts(MHP_ROBOT, -1);
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
///* mhpArmPlanTaskInter  -  codel INTER of ArmPlanTask
//   Returns:  INTER ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmPlanTaskInter(STRUCT_MHP_ARM_PLAN_PARAMS *armPlanTaskParams, int *report)
//{
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
///*------------------------------------------------------------------------
// * ArmSelectTraj
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// *              S_mhp_INVALID_TRAJ_ID
// *              S_mhp_NO_TRAJ
// */
//
///* mhpArmSelectTrajStart  -  codel START of ArmSelectTraj
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmSelectTrajStart(int *currentTrajId, int *report)
//{
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  int indexTraj = *currentTrajId;
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//  if(indexTraj < 0 || indexTraj > MHP_ARM_NB_MAX_TRAJ) {
//    *report = S_mhp_INVALID_TRAJ_ID;
//    return END;
//  }
//
//  return EXEC;
//}
//
///* mhpArmSelectTrajMain  -  codel EXEC of ArmSelectTraj
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmSelectTrajMain(int *currentTrajId, int *report)
//{
//  int indexTraj = *currentTrajId;
//
//  mhpArmTrajarmTrajectoryOuputPosterWrite( MHP_ARMTRAJ_POSTER_ID, &SDI_F->armTrajArray.traj[indexTraj]);
//  return END;
//}
//
///* mhpArmSelectTrajEnd  -  codel END of ArmSelectTraj
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmSelectTrajEnd(int *currentTrajId, int *report)
//{
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
///* mhpArmSelectTrajInter  -  codel INTER of ArmSelectTraj
//   Returns:  INTER ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmSelectTrajInter(int *currentTrajId, int *report)
//{
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
//
///*------------------------------------------------------------------------
// * ArmGetTrajInfo
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// *              S_mhp_INVALID_TRAJ_ID
// *              S_mhp_NO_TRAJ
// */
//
///* mhpArmGetTrajInfoStart  -  codel START of ArmGetTrajInfo
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmGetTrajInfoStart(int *currentTrajId, STRUCT_TRAJ_INFO *armTrajInfo, int *report)
//{
//   SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  int indexTraj = *currentTrajId;
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//  if(indexTraj < 0 || indexTraj > MHP_ARM_NB_MAX_TRAJ) {
//    *report = S_mhp_INVALID_TRAJ_ID;
//    return END;
//  }
//
//  return EXEC;
//}
//
///* mhpArmGetTrajInfoMain  -  codel EXEC of ArmGetTrajInfo
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmGetTrajInfoMain(int *currentTrajId, STRUCT_TRAJ_INFO *armTrajInfo, int *report)
//{
//  double RX=0.0, RY=0.0, RZ=0.0;
//   configPt qstart = NULL;
//  // mhpInitRobotPoseForPlanning
//  XYZ_ENV->cur_robot= MHP_ROBOT;
//  printf("your robot is %s\n",MHP_ROBOT->name);
//  qstart= p3d_alloc_config(MHP_ROBOT);
//
//  p3d_get_robot_config_into(MHP_ROBOT, &qstart);
//
//  p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qstart);
//
//
//
//  int result = 0;
//  int trajId = *currentTrajId;
//
//  //int indexTraj = SDI_F->armTrajArray.traj[trajId].nb_points -1;
//  //
//  //armTrajInfo->armQGoto.q1 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q1;
//  //armTrajInfo->armQGoto.q2 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q2;
//  //armTrajInfo->armQGoto.q3 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q3;
//  //armTrajInfo->armQGoto.q4 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q4;
//  //armTrajInfo->armQGoto.q5 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q5;
//  //armTrajInfo->armQGoto.q6 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q5;
//  //armTrajInfo->armQGoto.q7 =  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q7;
//
//  //MANIP_JIDO->setArmQ( SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q1,  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q2,
//  //      SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q3,  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q4,
//  //     SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q5,  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q6,  SDI_F->armTrajArray.traj[trajId].positions[indexTraj].q7);
//
//
//  //mhp_Arm2ConfigPt(SDI_F->armTrajArray.traj[trajId].positions[indexTraj], qstart);
//  //p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qstart);
//  //p3d_get_robot_config_into(MHP_ROBOT, &qstart);
//  //
//  //MANIP_JIDO->getArmX(&armTrajInfo->armXAbsGoto.x, &armTrajInfo->armXAbsGoto.y,&armTrajInfo->armXAbsGoto.z, &RX, &RY, &RZ);
//  p3d_destroy_config(MHP_ROBOT, qstart);
//
//  return END;
//}
//
///* mhpArmGetTrajInfoEnd  -  codel END of ArmGetTrajInfo
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmGetTrajInfoEnd(int *currentTrajId, STRUCT_TRAJ_INFO *armTrajInfo, int *report)
//{
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
///* mhpArmGetTrajInfoInter  -  codel INTER of ArmGetTrajInfo
//   Returns:  INTER ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmGetTrajInfoInter(int *currentTrajId, STRUCT_TRAJ_INFO *armTrajInfo, int *report)
//{
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
///*------------------------------------------------------------------------
// * ArmSetSupport
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// *              S_mhp_INVALID_SUPPORT
// */
//
///* mhpArmSetSupportMain  -  codel EXEC of ArmSetSupport
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmSetSupportMain(GEN_STRING128 *support, int *report)
//{
//
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//
////  if(MANIP_JIDO->setSupport(support->name)!=0) {
////    printf("ERROR cannot set the support to %s\n",support->name);
////    *report = S_mhp_INVALID_SUPPORT;
////  }
//
//  SDI_F->State.planning = FALSE;
//
//  return ETHER;
//}
//
//
///*------------------------------------------------------------------------
// * ArmCheckCollisionOnTraj
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// */
//
///* mhpArmCheckCollisionOnTrajStart  -  codel START of ArmCheckCollisionOnTraj
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmCheckCollisionOnTrajStart(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  GENMANIP_POSITION_STR poseOnTraj;
//
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//  if(mhpGENMANIP_POSITION_STRPosterFind(posterArmPoseOnTrajName->name, &posterArmPoseOnTraj)!=OK) {
//    *report = S_mhp_POSTER_NOT_FOUND;
//    return END;
//  }
//
//  if(mhpGENMANIP_POSITION_STRPosterRead(posterArmPoseOnTraj, &poseOnTraj)!=OK) {
//    *report = S_mhp_POSTER_NOT_READ;
//    return END;
//  }
//  SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//  SDI_F->collisionState.mode = GENMANIP_ON;
//  return EXEC;
//}
//
///* mhpArmCheckCollisionOnTrajMain  -  codel EXEC of ArmCheckCollisionOnTraj
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmCheckCollisionOnTrajMain(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  GENMANIP_POSITION_STR poseOnTraj;
//
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  if(mhpGENMANIP_POSITION_STRPosterRead(posterArmPoseOnTraj, &poseOnTraj)!=OK) {
//    *report = S_mhp_POSTER_NOT_READ;
//    return END;
//  }
//#ifdef DPG
//  if(MANIP_JIDO->checkCollisionOnTraj(poseOnTraj.lp)==0) {
//    SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//  } else {
//    SDI_F->collisionState.state = GENMANIP_COLLISION_COLLISION;
//    return END;
//    /*   MANIP_JIDO->replanCollidingTraj(MHP_ROBOT, 0, poseOnTraj->lp, );*/
//  }
//#else
//  printf("flag DPG is not defined\n");
//#endif
//
//  SDI_F->State.planning = FALSE;
//  return EXEC;
//}
//
///* mhpArmCheckCollisionOnTrajEnd  -  codel END of ArmCheckCollisionOnTraj
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmCheckCollisionOnTrajEnd(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
///*------------------------------------------------------------------------
// * ArmRePlanColTraj
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// */
//
///* mhpArmRePlanColTrajStart  -  codel START of ArmRePlanColTraj
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmRePlanColTrajStart(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//  GENMANIP_POSITION_STR poseOnTraj;
//
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//
//  if(mhpGENMANIP_POSITION_STRPosterFind(posterArmPoseOnTrajName->name, &posterArmPoseOnTraj)!=OK)
// {
//    *report = S_mhp_POSTER_NOT_FOUND;
//    return END;
//  }
///*  if(SDI_F->collisionState.state == GENMANIP_COLLISION_FREE) {
//    return END;
//  }*/
//  // printf("Replaning Start OK\n");
//  return EXEC;
//}
//
///* mhpArmRePlanColTrajMain  -  codel EXEC of ArmRePlanColTraj
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmRePlanColTrajMain(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//  GENMANIP_POSITION_STR poseOnTraj;
//  int indexTraj = 1;
//  int result;
//  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
//  std::vector <SM_TRAJ> smTrajs;
//
//  if(mhpGENMANIP_POSITION_STRPosterRead(posterArmPoseOnTraj, &poseOnTraj)!=OK) {
//    *report = S_mhp_POSTER_NOT_READ;
//    return END;
//  }
//  printf("Replanning\n");
//
//#ifdef DPG
//  result = MANIP_JIDO->replanCollidingTraj(poseOnTraj.lp, confs, smTrajs);
//
//  if(result!=0) {
//    *report= S_mhp_NO_PATH_FOUND;
//    return END;
//  }
//
//#else
//  printf("flag DPG is not defined\n");
//#endif
//
//
////  for(unsigned int i = 0; i < MANIP_JIDO->positions.size(); i++){
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q1 =  MANIP_JIDO->positions[i][0];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q2 =  MANIP_JIDO->positions[i][1];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q3 =  MANIP_JIDO->positions[i][2];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q4 =  MANIP_JIDO->positions[i][3];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q5 =  MANIP_JIDO->positions[i][4];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q6 =  MANIP_JIDO->positions[i][5];
////     SDI_F->armTrajArray.traj[indexTraj].positions[i].q7 =  MANIP_JIDO->positions[i][6];
////     SDI_F->armTrajArray.traj[indexTraj].lp[i] =  MANIP_JIDO->lp[i];
////  }
////  SDI_F->armTrajArray.traj[indexTraj].nb_points =  MANIP_JIDO->positions.size();
////  //printf("TRAJECTORY :\n");
////  //printf("nbPoints = %d\n",SDI_F->armTrajArray.traj[indexTraj].nb_points);
////  // for(int i = 0; i<SDI_F->armTrajArray.traj[indexTraj].nb_points; i++) {
////  //  printf("lpId %d q[%d] %f %f %f %f %f %f %f\n",  SDI_F->armTrajArray.traj[indexTraj].lp[i],i,
////  //	   SDI_F->armTrajArray.traj[indexTraj].positions[i].q1, SDI_F->armTrajArray.traj[indexTraj].positions[i].q2,
////  //   SDI_F->armTrajArray.traj[indexTraj].positions[i].q3, SDI_F->armTrajArray.traj[indexTraj].positions[i].q4,
////  //   SDI_F->armTrajArray.traj[indexTraj].positions[i].q5, SDI_F->armTrajArray.traj[indexTraj].positions[i].q6, SDI_F->armTrajArray.traj[indexTraj].positions[i].q7);
////
////  // }
////  mhpArmTrajarmTrajectoryOuputPosterWrite( MHP_ARMTRAJ_POSTER_ID, &SDI_F->armTrajArray.traj[indexTraj]);
//  return END;
//}
//
///* mhpArmRePlanColTrajEnd  -  codel END of ArmRePlanColTraj
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmRePlanColTrajEnd(GEN_STRING128 *posterArmPoseOnTrajName, int *report)
//{
//  SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
///*------------------------------------------------------------------------
// * ArmComputePRM
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_NOT_INITIALIZED
// *              S_mhp_WRONG_PARAMETER
// */
//
///* mhpArmComputePRMStart  -  codel START of ArmComputePRM
//   Returns:  START EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmComputePRMStart(double *prmComputationTime, int *report)
//{
//  configPt qstart= NULL;
//
//  if(!mhp_fetch_and_update_env(report))
//    printf("Warning: Cannot fetch the current environment\n");
//
//  SDI_F->State.planning = TRUE; /* --- Planning Mode ---- */
//
//  SDI_F->collisionState.state = GENMANIP_COLLISION_FREE;
//
//  if( *prmComputationTime < 0.0) {
//    // *report = S_mhp_WRONG_PARAMETER;
//    return END;
//  }
//
//  if (MANIP_JIDO== NULL) {
//    *report = S_mhp_NOT_INITIALIZED;
//    return END;
//  }
//
//  XYZ_ENV->cur_robot= MHP_ROBOT;
//  //  printf("your robot is %s\n",MHP_ROBOT->name);
//  qstart= p3d_alloc_config(MHP_ROBOT);
//  p3d_get_robot_config_into(MHP_ROBOT, &qstart);
//  p3d_update_virtual_object_config_for_arm_ik_constraint(MHP_ROBOT,0, qstart);
////  qstart[21] =  0.0325;
////  qstart[22] =  0.0325;
//  p3d_copy_config_into(MHP_ROBOT, qstart, &MHP_ROBOT->ROBOT_POS);
//  p3d_copy_config_into(MHP_ROBOT, qstart, &MHP_ROBOT->ROBOT_GOTO);
//  p3d_set_and_update_this_robot_conf(MHP_ROBOT, qstart);
//  p3d_destroy_config(MHP_ROBOT, qstart);
//
//  //  printf("ArmComputePRMStart OK\n");
//  return EXEC;
//}
//
///* mhpArmComputePRMMain  -  codel EXEC of ArmComputePRM
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmComputePRMMain(double *prmComputationTime, int *report)
//{
//  //MANIP_JIDO->setArmCartesian(false);
//
//
//  MANIP_JIDO->armComputePRM(*prmComputationTime);
//
//  return END;
//}
//
///* mhpArmComputePRMEnd  -  codel END of ArmComputePRM
//   Returns:  END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpArmComputePRMEnd(double *prmComputationTime, int *report)
//{
//
//  SDI_F->State.planning = FALSE;
//  return ETHER;
//}
//
//
//
//
//#endif
