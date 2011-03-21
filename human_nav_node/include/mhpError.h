
#ifndef mhp_ERROR_H
#define mhp_ERROR_H

// TODO, use H2 or something
//#include "h2errorLib.h"
//#include "genom/genomError.h"

/* M_id and err are encoded on 'signed short' (ie, < 2^15 = 32768) and 'short'
 *               S_lib_error = (M_lib << 16 | err)
 */
#define H2_ENCODE_ERR(M_id,err)   (M_id << 16 | (err&0xffff))

#define M_mhp    (1453)

/* -- MODULES ERRORS -------------------------------------------------- */

/* demo errors */
#define S_mhp_MHP_NOT_STOP         H2_ENCODE_ERR(M_mhp       , 1)
#define S_mhp_LOOPS_NOT_STOPPED    H2_ENCODE_ERR(M_mhp       , 2)
#define S_mhp_NO_INTERFACE_LAUCHED H2_ENCODE_ERR(M_mhp       , 3)
#define S_mhp_INTERFACE_DISABLED   H2_ENCODE_ERR(M_mhp       , 4)
#define S_mhp_NAV_BAD_ID           H2_ENCODE_ERR(M_mhp       , 5)
#define S_mhp_NAV_ALREADY_REACHED  H2_ENCODE_ERR(M_mhp       , 6)
#define S_mhp_NAV_INTERNAL_ERROR   H2_ENCODE_ERR(M_mhp       , 7)
#define S_mhp_NAV_NO_PATH_FOUND    H2_ENCODE_ERR(M_mhp       , 8)
#define S_mhp_NAV_HUMAN_TOO_CLOSE  H2_ENCODE_ERR(M_mhp       , 9)
#define S_mhp_NAV_GOAL_IN_OBSTACLE H2_ENCODE_ERR(M_mhp       , 10)
#define S_mhp_NAV_START_IN_OBSTACLE H2_ENCODE_ERR(M_mhp       , 11)
#define S_mhp_WRONG_PARAMETER      H2_ENCODE_ERR(M_mhp       , 12)
#define S_mhp_INVALID_SUPPORT      H2_ENCODE_ERR(M_mhp       , 13)
#define S_mhp_NO_TRAJ              H2_ENCODE_ERR(M_mhp       , 14)
#define S_mhp_EQUAL_QSTART_QGOAL   H2_ENCODE_ERR(M_mhp       , 15)
#define S_mhp_NO_PLACE             H2_ENCODE_ERR(M_mhp       , 16)
#define S_mhp_NO_GRASP             H2_ENCODE_ERR(M_mhp       , 17)
#define S_mhp_UNKNOWN_OBJECT       H2_ENCODE_ERR(M_mhp       , 18)
#define S_mhp_INVALID_TASK         H2_ENCODE_ERR(M_mhp       , 19)
#define S_mhp_INVALID_TRAJ_ID      H2_ENCODE_ERR(M_mhp       , 20)
#define S_mhp_INVALID_QGOAL        H2_ENCODE_ERR(M_mhp       , 21)
#define S_mhp_INVALID_QSTART       H2_ENCODE_ERR(M_mhp       , 22)
#define S_mhp_NO_TRAJ_FOUND        H2_ENCODE_ERR(M_mhp       , 23)
#define S_mhp_NOT_REACHED          H2_ENCODE_ERR(M_mhp       , 24)
#define S_mhp_OBJECT_UNKNOWN       H2_ENCODE_ERR(M_mhp       , 25)
#define S_mhp_NOT_REACHABLE        H2_ENCODE_ERR(M_mhp       , 26)
#define S_mhp_NO_PATH_FOUND        H2_ENCODE_ERR(M_mhp       , 27)
#define S_mhp_IS_PLAYING_A_TRAJ    H2_ENCODE_ERR(M_mhp       , 28)
#define S_mhp_CANNOT_FIND_TRAJ     H2_ENCODE_ERR(M_mhp       , 29)
#define S_mhp_CANNOT_FIND_ENVIRONMENT H2_ENCODE_ERR(M_mhp       , 30)
#define S_mhp_CANNOT_FIND_ROBOT    H2_ENCODE_ERR(M_mhp       , 31)
#define S_mhp_OBJECT_NOT_FOUND     H2_ENCODE_ERR(M_mhp       , 32)
#define S_mhp_INTERNAL_ERROR       H2_ENCODE_ERR(M_mhp       , 33)
#define S_mhp_UNKNOWN_HUMAN_STATE  H2_ENCODE_ERR(M_mhp       , 34)
#define S_mhp_AGENT_NOT_FOUND      H2_ENCODE_ERR(M_mhp       , 35)
#define S_mhp_POSTER_NOT_READ      H2_ENCODE_ERR(M_mhp       , 36)
#define S_mhp_POSTER_NOT_COMPATIBLE H2_ENCODE_ERR(M_mhp       , 37)
#define S_mhp_POSTER_NOT_FOUND     H2_ENCODE_ERR(M_mhp       , 38)
#define S_mhp_MHP_P3D_LOAD_TWICE   H2_ENCODE_ERR(M_mhp       , 39)
#define S_mhp_NAV_NOT_INITIALIZED  H2_ENCODE_ERR(M_mhp       , 40)
#define S_mhp_ERROR_UNKNOWN        H2_ENCODE_ERR(M_mhp       , 41)
#define S_mhp_GIK_NOT_CREATED      H2_ENCODE_ERR(M_mhp       , 42)
#define S_mhp_FILE_NOT_FOUND       H2_ENCODE_ERR(M_mhp       , 43)
#define S_mhp_NOT_INITIALIZED      H2_ENCODE_ERR(M_mhp       , 44)
#define S_mhp_MHP_INIT_FAILED      H2_ENCODE_ERR(M_mhp       , 45)


/* std errors */
#define S_mhp_stdGenoM_ACTIVITY_INTERRUPTED       95282177   /* 1453<<16 | 0x8000 | 100<<8 | 1 */
#define S_mhp_stdGenoM_TOO_MANY_ACTIVITIES        95282178   /* 1453<<16 | 0x8000 | 100<<8 | 2 */
#define S_mhp_stdGenoM_ACTIVITY_FAILED            95282179   /* 1453<<16 | 0x8000 | 100<<8 | 3 */
#define S_mhp_stdGenoM_WAIT_ABORT_ZOMBIE_ACTIVITY 95282180   /* 1453<<16 | 0x8000 | 100<<8 | 4 */
#define S_mhp_stdGenoM_UNKNOWN_ACTIVITY           95282181   /* 1453<<16 | 0x8000 | 100<<8 | 5 */
#define S_mhp_stdGenoM_FORBIDDEN_ACTIVITY_TRANSITION 95282182   /* 1453<<16 | 0x8000 | 100<<8 | 6 */
#define S_mhp_stdGenoM_SYSTEM_ERROR               95282183   /* 1453<<16 | 0x8000 | 100<<8 | 7 */
#define S_mhp_stdGenoM_ACTIVITY_ALREADY_ENDED     95282184   /* 1453<<16 | 0x8000 | 100<<8 | 8 */
#define S_mhp_stdGenoM_WAIT_INIT_RQST             95282185   /* 1453<<16 | 0x8000 | 100<<8 | 9 */
#define S_mhp_stdGenoM_CONTROL_CODEL_ERROR        95282186   /* 1453<<16 | 0x8000 | 100<<8 | 10 */
#define S_mhp_stdGenoM_EXEC_TASK_SUSPENDED        95282187   /* 1453<<16 | 0x8000 | 100<<8 | 11 */
#define S_mhp_stdGenoM_BAD_BLOCK_TYPE             95282188   /* 1453<<16 | 0x8000 | 100<<8 | 12 */
#define S_mhp_stdGenoM_BAD_POSTER_TYPE            95282189   /* 1453<<16 | 0x8000 | 100<<8 | 13 */



/* static H2_ERROR[] */
#define MHP_H2_ERR_MSGS {\
    {"MHP_NOT_STOP", 1}, \
    {"LOOPS_NOT_STOPPED", 2},\
    {"NO_INTERFACE_LAUCHED", 3},\
    {"INTERFACE_DISABLED", 4},\
    {"NAV_BAD_ID", 5},\
    {"NAV_ALREADY_REACHED", 6},\
    {"NAV_INTERNAL_ERROR", 7},\
    {"NAV_NO_PATH_FOUND", 8},\
    {"NAV_HUMAN_TOO_CLOSE", 9},\
    {"NAV_GOAL_IN_OBSTACLE", 10},\
    {"NAV_START_IN_OBSTACLE", 11},\
    {"WRONG_PARAMETER", 12},\
    {"INVALID_SUPPORT", 13},\
    {"NO_TRAJ", 14},\
    {"EQUAL_QSTART_QGOAL", 15},\
    {"NO_PLACE", 16},\
    {"NO_GRASP", 17},\
    {"UNKNOWN_OBJECT", 18},\
    {"INVALID_TASK", 19},\
    {"INVALID_TRAJ_ID", 20},\
    {"INVALID_QGOAL", 21},\
    {"INVALID_QSTART", 22},\
    {"NO_TRAJ_FOUND", 23},\
    {"NOT_REACHED", 24},\
    {"OBJECT_UNKNOWN", 25},\
    {"NOT_REACHABLE", 26},\
    {"NO_PATH_FOUND", 27},\
    {"IS_PLAYING_A_TRAJ", 28},\
    {"CANNOT_FIND_TRAJ", 29},\
    {"CANNOT_FIND_ENVIRONMENT", 30},\
    {"CANNOT_FIND_ROBOT", 31},\
    {"OBJECT_NOT_FOUND", 32},\
    {"INTERNAL_ERROR", 33},\
    {"UNKNOWN_HUMAN_STATE", 34},\
    {"AGENT_NOT_FOUND", 35},\
    {"POSTER_NOT_READ", 36},\
    {"POSTER_NOT_COMPATIBLE", 37},\
    {"POSTER_NOT_FOUND", 38},\
    {"MHP_P3D_LOAD_TWICE", 39},\
    {"NAV_NOT_INITIALIZED", 40},\
    {"ERROR_UNKNOWN", 41},\
    {"GIK_NOT_CREATED", 42},\
    {"FILE_NOT_FOUND", 43},\
    {"NOT_INITIALIZED", 44},\
    {"MHP_INIT_FAILED", 45},\
}

#ifdef __cplusplus
extern "C" {
#endif

extern int mhpRecordH2errMsgs(void);

#ifdef __cplusplus
}
#endif


/*-------------------------- end file loading ---------------------------*/
#endif
