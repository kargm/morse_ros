#include "mhpm3d.h"
#include <display.h>
#include <mhpError.h>
#include <globalVars.h>

//#ifdef HRP2
//void mhp_Arm2ConfigPt(ghrp2_config_t *cfg, configPt m3dconfig)
//{
//   int i;
//
//  for(i=0; i<16 ; i++)
//    m3dconfig[ MHP_Q_RLEG0 + i ] = cfg->angles[i];
//
//  for(i=0; i<7 ; i++)
//    m3dconfig[ MHP_Q_RARM0 + i ] = cfg->angles[ GHRP2_RARM_JOINT0 + i];
//
//  for(i=0; i<7 ; i++)
//    m3dconfig[ MHP_Q_LARM0 + i ] = cfg->angles[ GHRP2_LARM_JOINT0 + i];
//}
//#elif defined(JIDO)
//void mhp_Arm2ConfigPt(Gb_q7 *gbconfig, configPt m3dconfig)
//{
//  m3dconfig[14] = gbconfig->q1;
//  m3dconfig[15] = gbconfig->q2;
//  m3dconfig[16] = gbconfig->q3;
//  m3dconfig[17] = gbconfig->q4;
//  m3dconfig[18] = gbconfig->q5;
//  m3dconfig[19] = gbconfig->q6;
//  m3dconfig[20] = gbconfig->q7;
//}
//
//static void mhp_pt2ConfigPt(double pan, double tilt,  configPt m3dconfig)
//{
//  m3dconfig[12]  = pan;
//  m3dconfig[13]  = -tilt;
//}
//#else
//static void mhp_Arm2ConfigPt(void *gbconfig, void *m3dconfig)
//{}
//static void mhp_pt2ConfigPt( double pan, double tilt,  configPt m3dconfig)
//{}
//#endif
//
//static void mhp_xyz2ConfigPt( double x, double y, double z, double rz, configPt m3dconfig)
//{
//  m3dconfig[6]  = x;
//  m3dconfig[7]  = y;
//  m3dconfig[8]  = z;
//  m3dconfig[11] = rz;
//}

/* ------------------------------------------------------------- */
/* ----------------------- PLACE HUMAN ------------------------- */

/* mhpPlaceAgentMain  -  codel EXEC of PlaceAgent
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
mhpPlaceAgentMain(MHP_AGENT_POSITION *addedAgent, int *report)
{
  configPt agentq;
//  p3d_env * env;
  p3d_rob * rob = NULL;
  int is_human = FALSE;
  int i;
  *report = OK;



  if(MHP_GIK == NULL || MHP_ENV == NULL){
    printf("Agent or Environment not initialized in internal structures!\n");
    *report = S_mhp_NOT_INITIALIZED;
    return *report;
  }

  for(i=0; i<MHP_ENV->nr; i++){
    if( strcasestr(MHP_ENV->robot[i]->name, addedAgent->name.name) ){
      if( strcasestr(MHP_ENV->robot[i]->name,"HUMAN")){
	is_human = TRUE;
      }
      rob = MHP_ENV->robot[i];
      break;
    }
  }
  if(i == MHP_ENV->nr){
    *report = S_mhp_AGENT_NOT_FOUND;
    printf("Warning: Can't find the Agent %s\n", addedAgent->name.name);
    return *report;
  }

  if(is_human){
    // IF the agent is a human
    printf("NECK PAN AND TILT JOINTS: %d %d\n",HUMANj_NECK_PAN,HUMANj_NECK_TILT);

    if(addedAgent->state < 0 || addedAgent->state >= BT_STATE_NO){
      *report = S_mhp_UNKNOWN_HUMAN_STATE;
      printf("Can't add human: Unknown state: %d\n", addedAgent->state);
      return *report;
    }
    for(i=0; i<MHP_GIK->human_no; i++){
      if(rob == MHP_GIK->human[i]->HumanPt){

	MHP_GIK->human[i]->exists = TRUE;

	agentq = MY_ALLOC(double, MHP_GIK->human[i]->HumanPt->nb_dof); /* ALLOC */

	p3d_get_robot_config_into(MHP_GIK->human[i]->HumanPt, &agentq);

	agentq[6]  = addedAgent->pos.x; // x
	agentq[7]  = addedAgent->pos.y; // y
	agentq[11] = DTOR(addedAgent->pos.th); // Rz

	agentq[rob->joints[HUMANj_NECK_PAN]->index_dof] = DTOR(addedAgent->pan);
	agentq[rob->joints[HUMANj_NECK_TILT]->index_dof] = DTOR(addedAgent->tilt);

	if (!hri_set_human_state(MHP_GIK->human[i], addedAgent->state, agentq)) {
	  printf("Warning: Error when setting human state: %d\n", addedAgent->state);
	}

	p3d_set_and_update_this_robot_conf(MHP_GIK->human[i]->HumanPt,agentq);

	MY_FREE(agentq,double,MHP_GIK->human[i]->HumanPt->nb_dof); /* FREE */

	break;
      }
    }
    if(i==MHP_GIK->human_no){
      *report = S_mhp_INTERNAL_ERROR;
      return *report;
    }
  }
  else{
    // IF the agent is a robot
    agentq = MY_ALLOC(double, rob->nb_dof); /* ALLOC */

    p3d_get_robot_config_into(rob, &agentq);

    agentq[6]  = addedAgent->pos.x; // x
    agentq[7]  = addedAgent->pos.y; // y
    agentq[11] = DTOR(addedAgent->pos.th); // Rz

    agentq[rob->joints[ROBOTj_PAN]->index_dof] = DTOR(addedAgent->pan);
    agentq[rob->joints[ROBOTj_TILT]->index_dof] = DTOR(addedAgent->tilt);

    p3d_set_and_update_this_robot_conf(rob, agentq);

    MY_FREE(agentq,double,rob->nb_dof); /* FREE */

  }

  return OK;
}
  

///* ------------------------------------------------------------- */
///* ---------------------- PLACE OBJECT ---------------------- */
//
///* mhpPlaceObjectMain  -  codel EXEC of PlaceObject
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpPlaceObjectMain(MHP_M3D_OBJECT *addedObject, int *report)
//{
//  configPt robotq;
//  p3d_env * env;
//  int i;
//  p3d_rob * rob = NULL;
//
//  env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
//  for(i=0; i<env->nr; i++){
//    if( strcasestr(env->robot[i]->name,addedObject->name.name) ){
//      rob = env->robot[i];
//      break;
//    }
//  }
//
//  if( rob == NULL ){
//    *report = S_mhp_OBJECT_NOT_FOUND;
//    return ETHER;
//  }
//
//  robotq = MY_ALLOC(double, rob->nb_dof); /* ALLOC */
//
//  p3d_get_robot_config_into(rob, &robotq);
//
//  robotq[6] = addedObject->coords.x;
//  robotq[7] = addedObject->coords.y;
//  robotq[8] = addedObject->coords.z;
//  robotq[9] = addedObject->coords.yaw;
//  robotq[10] = addedObject->coords.pitch;
//  robotq[11] = addedObject->coords.roll;
//
//  p3d_set_and_update_this_robot_conf(rob,robotq);
//
//  MY_FREE(robotq,double,rob->nb_dof); /* FREE */
//
//  return ETHER;
//}
//
///* mhpGetCurRobotConfigMain  -  codel EXEC of GetCurRobotConfig
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpGetCurRobotConfigMain(int *report)
//{
//  configPt q = NULL;
//  q = p3d_get_robot_config(MHP_ROBOT);
//  printf("conf cur\n");
//  print_config(MHP_ROBOT, q);
//  printf("conf ROBOT_POS\n");
//  print_config(MHP_ROBOT, MHP_ROBOT->ROBOT_POS);
//  p3d_destroy_config(MHP_ROBOT, q);
//  return ETHER;
//}
//
///* -------------------------------------------------------------- */
//

static int
drawtraj(p3d_rob* robot, p3d_localpath* curLp){
  g3d_draw_allwin_active();
}

///* mhpShowCurTrajMain  -  codel EXEC of GetCurRobotConfig
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//
//static bool isPlayingTraj = false;
//ACTIVITY_EVENT
//mhpShowCurTrajMain(int *report){
//  if(!isPlayingTraj){
//    isPlayingTraj = true;
//    if(MHP_ROBOT->tcur == NULL){
//      *report = S_mhp_CANNOT_FIND_TRAJ;
//      return FAIL;
//    }
//    g3d_show_tcur_rob(MHP_ROBOT, drawtraj);
//  }else{
//    *report = S_mhp_IS_PLAYING_A_TRAJ;
//    return ETHER;
//  }
//  return END;
//}
//
///* mhpShowCurTrajEnd  -  codel EXEC of GetCurRobotConfig
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//
//ACTIVITY_EVENT
//mhpShowCurTrajEnd(int *report){
//  isPlayingTraj = true;
//  return ETHER;
//}
//
///*----------------------------------------------------------------
// * FindBestOTP
// *
// * Description:
// *
// * Reports:      OK
// *              S_mhp_ROBOT_NOT_FOUND
// *              S_mhp_OTP_NOT_FOUND
// *              S_mhp_NOT_INITIALIZED
// */
//
///* mhpFindBestOTPMain  -  codel EXEC of GetJointAbsPose
//   Returns:  EXEC END ETHER FAIL ZOMBIE */
//ACTIVITY_EVENT
//mhpFindBestOTPMain(GEN_STRING128 *humanName, POM_EULER *absPose, int *report)
//{
//
//#ifndef HRI_COSTSPACE
//  printf("MHP is not compiled for OTP support.\n");
//  *report = S_mhp_NOT_INITIALIZED;
//  return ETHER;
//#else
//
//  if(XYZ_ENV == NULL) {
//    printf("MHP is not properly initialized.\n");
//    *report = S_mhp_NOT_INITIALIZED;
//    return ETHER;
//  }
//
//  p3d_rob* rob = p3d_get_robot_by_name(humanName->name);
//
//  if( rob == NULL ) {
//    *report = S_mhp_ROBOT_NOT_FOUND;
//    return ETHER;
//  }
//
//  bool succed=false;
//  bool move_human=false;
//  int otp_type = 0;
//  Eigen::Vector3d WSPoint;
//
//  succed = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->ComputeTheObjectTransfertPoint(move_human,otp_type,WSPoint);
//
//  if( !succed ) {
//    *report = S_mhp_OTP_NOT_FOUND;
//    return ETHER;
//  }
//
//  absPose->x      = WSPoint[0];
//  absPose->y      = WSPoint[1];
//  absPose->z      = WSPoint[2];
//
//  // Convert Move3D abs_pos to Euler (Yaw,Pitch,Roll)
//  /*T3D* jointToOrigin = t3dNew( T3D_MATRIX, T3D_ALLOW_CONVERSION );
//
//  for(unsigned int i=0; i<4; i++) {
//    for(unsigned int j=0; j<4; j++) {
//      jointToOrigin->matrix[4*i+j] = jnt->abs_pos[i][j];
//      }
//   }
//
//  t3dConvertTo( T3D_BRYAN, jointToOrigin );
//
//  jointAbsPose->x      = jointToOrigin->euler.x;
//  jointAbsPose->y      = jointToOrigin->euler.y;
//  jointAbsPose->z      = jointToOrigin->euler.z;
//  jointAbsPose->yaw    = jointToOrigin->euler.yaw;
//  jointAbsPose->pitch  = jointToOrigin->euler.pitch;
//  jointAbsPose->roll   = jointToOrigin->euler.roll;
//
//
//  delete jointToOrigin;*/
//
//  //  p3d_mat4ExtractPosReverseOrder(M,
//  //				 &(jointAbsPose->x), &jointAbsPose->y, jointAbsPose->z,
//  //				 &(jointAbsPose->roll), &(jointAbsPos->pitch),&(
//
//  return ETHER;
//
//#endif
//}

