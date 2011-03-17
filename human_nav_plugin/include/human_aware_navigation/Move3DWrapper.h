/*
 * Move3DWrapper.h
 *
 *  Created on: May 14, 2009
 *      Author: kruset
 */

#ifndef MOVE3DWRAPPER_H_
#define MOVE3DWRAPPER_H_

typedef enum STRUCT_HUMAN_STATE_ENUM{
  // must be the same as BT_... constants in Move3d hri_planner
  STANDING = 0,
  SITTING = 1,
  MOVING = 2,
  STANDING_TRANSPARENT = 3// if planner may plan through this human
} HUMAN_STATE_ENUM;

/**
  * A structure to capture all known / relevant humans in the environment
  */
typedef struct Human
 {
   double x;
   double y;

   double az; // head rotation

   int pose; // position (standing sitting moving) as int
   int locked; // whether human will not move out of the way
   int exists; // 0 if not
   long lastMoveTimeStampSecs; // used to determine if human is moving
 } Human;

struct Waypoint
{
	double x;
	double y;
	double az; // ignore for waypoint calculation result
};

#define MXR_REQUEST_PATH_METHOD_NAME "MhpXmlRpc.requestPath"
#define MXR_STOP_SERVER "MhpXmlRpc.stopServer"

/**
 * !! Must be <= PILO_MAX_TRAJ_SEGMENTS in piloStruct.h fromMove3D!
 */
#define M3DW_MAX_ABSOLUTE_TRAJECTORY_SEGMENTS 50

#define MHPD_MAX_HUMANS 5
#define M3DW_MAX_WP_DIST 0.8

/* whether to show graphics display while running M3d or not */
#define M3DW_MH_SHOW_DISPLAY 1

typedef struct AbsoluteTrajectory
{
  int numberOfSegments;
  double costs;
  struct Waypoint seg[M3DW_MAX_ABSOLUTE_TRAJECTORY_SEGMENTS];
} AbsoluteTrajectory;


/** starting errors in the 100s to avoid error collision with m3d reports*/
#define m3dw_ERROR_CLIENT_NOT_INITIALIZED -101
#define m3dw_ERROR_UNEXPECTED_TRAJ_RESULT -102
#define m3dw_ERROR_NAV_NOT_INITIALISED -103
#define m3dw_ERROR_POSITION_REACHED -104
#define m3dw_ERROR_HUMAN_TOO_CLOSE -105
#define m3dw_ERROR_MHP_INTERNAL_ERROR -106
#define m3dw_ERROR_NO_PATH_FOUND -107
#define m3dw_ERROR_GOAL_IN_OBSTACLE -108
#define m3dw_ERROR_START_IN_OBSTACLE -109
#define m3dw_ERROR_GENOM -110
#define m3dw_ERROR_MEMFAULT -111

#endif /* MOVE3DWRAPPER_H_ */
