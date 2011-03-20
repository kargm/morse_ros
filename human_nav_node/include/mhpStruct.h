/*****************************************************************************
*
* Emrah Akin Sisbot
* November 06
*
* File    : mhpStruct.h
* Brief   :
*
******************************************************************************/

#ifndef MHP_STRUCT_H
#define MHP_STRUCT_H

/*----------------------------------------------------------------------------
  INCLUDES
-----------------------------------------------------------------------------*/

#include "mhpConst.h"
#include "genBasicStruct.h"

/*----------------------------------------------------------------------------
  STRUCTURES
-----------------------------------------------------------------------------*/
typedef struct STRUCT_MHP_P3D{
  GEN_STRING128 P3dModeleName;
  int enableGraphic;
  int enablePersp;
} MHP_P3D;

typedef struct  STRUCT_MHP_SCE{
  GEN_STRING128 SceFileName;
} MHP_SCE;

typedef struct STRUCT_MHP_STATE{
  int posPosterAv;
  int armConfPosterAv;
  int handPosterAv;
  int ptPosterAv;
  int objPosPosterAv;
  int hrp2ConfPosterAv;
  int mocapPosterAv;
  int morsePosterAv;
  int planning;
  int debug_mode;
  int oroConnectionAv;
  int updating;
  int envPosterAv;
  int unused;
} MHP_STATE;

typedef struct STRUCT_MHP_NAVIGATION_PARAMS {
  double path_segment_length;
  double grid_sampling;
  double follow_distance;
} MHP_NAVIGATION_PARAMS;

typedef struct STRUCT_MHP_INTERFACE_PARAMS{
  int width;
  int height;
  GEN_BOOL saveInterface;
  GEN_BOOL show_nav_obstacles;
  GEN_BOOL show_nav_path;
  GEN_BOOL show_nav_distance_grid;
  GEN_BOOL show_nav_visibility_grid;
  GEN_BOOL show_nav_hidzones_grid;
  GEN_BOOL floor;
  GEN_BOOL walls;
  GEN_BOOL tiles;
  GEN_BOOL shadows;
  GEN_BOOL draw_graph;
  int      draw_graph_jnt_id;
} MHP_INTERFACE_PARAMS;

typedef struct STRUCT_MHP_INTERFACE_STATE{
  GEN_BOOL savingInterface;
  GEN_BOOL updating;
  GEN_BOOL showing_nav_obstacles;
  GEN_BOOL showing_nav_path;
  GEN_BOOL showing_nav_distance_grid;
  GEN_BOOL showing_nav_visibility_grid;
  GEN_BOOL showing_nav_hidzones_grid;
  GEN_BOOL showing_floor;
  GEN_BOOL showing_walls;
  GEN_BOOL showing_tiles;
  GEN_BOOL showing_shadows;
  GEN_BOOL unUsed;
} MHP_INTERFACE_STATE;


typedef struct STRUCT_MHP_VISIBILITY_PARAMS{
  double FoA_h;
  double FoA_v;
  double FoV_h;
  double FoV_v;
  double Vis_Ratio;
} MHP_VISIBILITY_PARAMS;

typedef struct STRUCT_MHP_INTERFACE_AGENT_PARAMS {
  GEN_STRING64 agent_name;
  int show_fov;
  int show_poiting;
} MHP_INTERFACE_AGENT_PARAMS;

//---- Input posters --------

typedef struct STRUCT_MHP_ROBOT_CONF_POSTERS {
  GEN_STRING64 armConfPostername;
  GEN_STRING64 posPostername; 
  GEN_STRING64 ptPostername;
} MHP_ROBOT_CONF_POSTERS;

typedef enum STRUCT_MHP_HUMAN_POSTER_TYPE_ENUM{
  MHP_USE_MOCAP = 0,
  MHP_USE_GEST = 1,
  MHP_USE_HUMPOS = 2,
  MHP_USE_MORSE = 3
} MHP_HUMAN_POSTER_TYPE_ENUM;

typedef struct STRUCT_MHP_HUMAN_POSTERS{
  MHP_HUMAN_POSTER_TYPE_ENUM posterType;
  int unused;
  GEN_STRING64 postername;
} MHP_HUMAN_POSTERS;

typedef struct STRUCT_MHP_HUMAN_PARAMS{
  double MHP_HUMAN_UPDATE_DIST;
  double MHP_HUMAN_UPDATE_ANGLE;
} MHP_HUMAN_PARAMS;

typedef struct STRUCT_MHP_HUMANS{
  MHP_HUMAN_PARAMS Human_Param;
  int Tracked_Human_no;
  int Tracked_psp_human_no;
} MHP_HUMANS;

typedef struct STRUCT_MHP_OBJECT_POSTERS{
  GEN_STRING64 Objectpostername;
} MHP_OBJECT_POSTERS;

typedef struct STRUCT_MHP_OBJECT_PARAMS{
  double MHP_OBJECT_UPDATE_DIST;
  double MHP_OBJECT_UPDATE_ANGLE;
} MHP_OBJECT_PARAMS;

typedef struct STRUCT_MHP_OBJECTS{
  MHP_OBJECT_POSTERS posters;
  MHP_OBJECT_PARAMS params;
} MHP_OBJECTS;

// ---------------------

typedef struct STRUCT_MHP_CAM_POS{
  double xdest;
  double ydest;
  double zdest;
  double dist;
  double hrot;
  double vrot;
} MHP_CAM_POS;

typedef enum STRUCT_MHP_MOTION_TYPE_ENUM{
  MHP_LOOK = 0,
  MHP_LEFT_HAND_POINT = 1,
  MHP_RIGHT_HAND_POINT = 2,
  MHP_LEFT_HAND_REACH = 3,
  MHP_RIGHT_HAND_REACH = 4
} MHP_MOTION_TYPE_ENUM;

typedef struct STRUCT_MHP_6D_COORD{
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
} MHP_6D_COORD;

typedef struct STRUCT_MHP_MOTION_DATA{
  MHP_MOTION_TYPE_ENUM motiontype;
  GEN_STRING128 target;
  MHP_6D_COORD coords;
} MHP_MOTION_TARGET;

typedef struct STRUCT_MHP_3D_XYTH{
  double x;
  double y;
  double th;
} MHP_3D_XYTH;

typedef struct STRUCT_MHP_3D_XYZ{
  double x;
  double y;
  double z;
} MHP_3D_XYZ;

typedef struct STRUCT_MHP_MOTION_TASK_DATA{
  MHP_MOTION_TYPE_ENUM motiontype;
  GEN_STRING128 target;
} MHP_MOTION_TASK_DATA;

typedef struct STRUCT_MHP_AGENT_MOTION_TASK {
  GEN_STRING128 name;
  MHP_MOTION_TASK_DATA task;
} MHP_AGENT_MOTION_TASK;

typedef struct STRUCT_MHP_AGENT_MOTION_TASK_RESULT {
  double look[4];
  int look_nb;
  double arm[7];
  int arm_nb;
} MHP_AGENT_MOTION_TASK_RESULT;

typedef enum STRUCT_MHP_HUMAN_STATE_ENUM{
  // must be the same as BT_... constants in Move3d hri_planner
  MHP_STANDING = 0,
  MHP_SITTING = 1,
  MHP_MOVING = 2,
  MHP_STANDING_TRANSPARENT = 3// if planner may plan through this human
} MHP_HUMAN_STATE_ENUM;

typedef struct STRUCT_MHP_HUMAN_POSITION{
  MHP_3D_XYTH pos;
  int id;  
  MHP_HUMAN_STATE_ENUM state;
} MHP_HUMAN_POSITION;

typedef struct STRUCT_MHP_AGENT_POSITION{
  GEN_STRING64 name;
  MHP_3D_XYTH pos;
  double pan;
  double tilt;
  int state;
  int nouse;
} MHP_AGENT_POSITION;

// poster struct giving the last beliefs about positions and stae of agents, to be replaced with ontology
typedef struct STRUCT_MHP_AGENTS_POSITIONS{
  MHP_AGENT_POSITION human[MHP_MAX_AGENT_NB];
  int lhandstate[MHP_MAX_AGENT_NB];
  int rhandstate[MHP_MAX_AGENT_NB];
} MHP_AGENTS_POSITIONS;

typedef struct STRUCT_MHP_REACHABILITY_QUERY{
  GEN_STRING128 string1;
  GEN_STRING128 string2;
  int vis_step;
  int unused;
} MHP_REACHABILITY_QUERY;

typedef struct STRUCT_MHP_VISIBILITY_QUERY{
  GEN_STRING128 string1;
  GEN_STRING128 string2;
} MHP_VISIBILITY_QUERY;

typedef struct STRUCT_MHP_TWO_STRINGS{
  GEN_STRING128 string1;
  GEN_STRING128 string2;
} MHP_TWO_STRINGS;
typedef struct STRUCT_MHP_THREE_STRINGS{
  GEN_STRING128 string1;
  GEN_STRING128 string2;
  GEN_STRING128 string3;
} MHP_THREE_STRINGS;

typedef struct STRUCT_MHP_M3D_OBJECT{
  GEN_STRING128 name;
  MHP_6D_COORD coords;
} MHP_M3D_OBJECT;

typedef enum ENUM_MHP_OBJECT_MOTION{
  MHP_OBJ_NONE = 0,
  MHP_OBJ_STATIC = 1,
  MHP_OBJ_MOVING = 2
} MHP_OBJECT_MOTION;

/* ---------- Poster Structures --------------- */

typedef struct STRUCT_MHP_SPRELATION {
  GEN_STRING128 target;
  GEN_STRING128 reference;
  int sp_rel_val;
  int nouse;
} MHP_SPRELATION;

typedef struct STRUCT_MHP_SPRELATIONS {
  int id;
  int length;
  MHP_SPRELATION relation[MHP_SPREL_MAX];
} MHP_SPRELATIONS;

typedef struct STRUCT_MHP_ENVDEF {
  int id; //Not used
  int length;
  GEN_STRING128 environment;
  GEN_STRING128 things[MHP_MAX_NO_THINGS]; 
} MHP_ENVDEF;

typedef struct STRUCT_MHP_PLRELATIONS {
  int id;
  int length;
  MHP_SPRELATION relation[MHP_SPREL_MAX];
} MHP_PLRELATIONS;

typedef enum ENUM_MHP_VISIBILITY {
  MHP_INVISIBLE = 0,
  MHP_VISIBLE = 1
} MHP_VISIBILITY;

typedef enum ENUM_MHP_VISIBILITY_PLACEMENT {
  MHP_OOF = 3,
  MHP_FOV = 2,
  MHP_FOA = 1
} MHP_VISIBILITY_PLACEMENT;

typedef enum ENUM_MHP_REACHABILITY {
  MHP_UNREACHABLE = 0,
  MHP_REACHABLE = 1,
  MHP_REACHABLE_WITH_COLLISION = 2
} MHP_REACHABILITY;

typedef struct STRUCT_MHP_AGENT_REACHABILITY {
  GEN_STRING128 agent_name;
  GEN_STRING128 object_name;
  MHP_REACHABILITY reachability;
} MHP_AGENT_REACHABILITY;

typedef struct STRUCT_MHP_ALL_REACHABILITIES {
  MHP_AGENT_REACHABILITY *agent_reach;
  int agent_reach_nb;
} MHP_ALL_REACHABILITIES;

typedef enum ENUM_MHP_PLRELATION {
  MHP_ISIN = 0,
  MHP_ISON = 1,
  MHP_ISNEXTTO = 2,
  MHP_NOPLR = 3
} MHP_PLRELATION;

typedef struct STRUCT_MHP_SYM_PLACEMENT {
  MHP_PLRELATION relation;
  GEN_STRING128 object;
} MHP_SYM_PLACEMENTS;

typedef struct STRUCT_MHP_VISIBILITY_STATE {
  GEN_STRING128 agent_name;
  MHP_VISIBILITY Visibility;
  MHP_VISIBILITY_PLACEMENT VisibilityPlacement;
  int pointedAt;
  int spatialRelation;
} MHP_VISIBILITY_STATE;

typedef struct STRUCT_MHP_OBJECT_STATE {
  GEN_STRING128  name;
  unsigned long last_time_seen;
  unsigned long last_time_static;
  int detected;
  int disappeared;
  MHP_6D_COORD pos;
  MHP_OBJECT_MOTION motion;
  MHP_VISIBILITY_STATE AgentVisibilityState[MHP_MAX_AGENT_NB];
  int AgentVisibilityStateNb;
  MHP_REACHABILITY robotReachability;
  MHP_REACHABILITY humanReachability;
  MHP_SYM_PLACEMENTS placements[MHP_OBJ_STATE_MAX];
  int placement_length;
} MHP_OBJECT_STATE;

typedef struct STRUCT_MHP_OBJECT_STATE_ARRAY {
  MHP_OBJECT_STATE ObjectStates[MHP_OBJ_STATE_MAX];
  int length;
} MHP_OBJECT_STATE_ARRAY;


/**
 * one reading of several agent headpositions
 */
typedef struct STRUCT_MHP_AGENT_HEAD_READING {
  MHP_6D_COORD head;
  unsigned long sensor_time_head;
} MHP_AGENT_HEAD_READING;

typedef struct STRUCT_MHP_AGENT_HAND_READING {
  MHP_3D_XYZ hand;
  unsigned long sensor_time_head;
} MHP_AGENT_HAND_READING;


typedef struct STRUCT_MHP_AGENT_POSITION_HISTORY_ARRAY {
   MHP_6D_COORD head[MHP_HUMAN_HISTORY_LENGTH];
   MHP_3D_XYZ lhand[MHP_HUMAN_HISTORY_LENGTH];
   MHP_3D_XYZ rhand[MHP_HUMAN_HISTORY_LENGTH];
   unsigned long sensor_time_head[MHP_HUMAN_HISTORY_LENGTH];
   int head_length;
   int lhand_length;
   int rhand_length;
} MHP_AGENT_POSITION_HISTORY_ARRAY;

typedef struct STRUCT_MHP_AGENT_HISTORY_UNIT {
  MHP_6D_COORD head;
  MHP_3D_XYZ lhand;
  MHP_3D_XYZ rhand;
  unsigned long sensor_time_head;
} MHP_AGENT_HISTORY_UNIT;

typedef struct STRUCT_MHP_AGENT_HISTORY {
  MHP_AGENT_HISTORY_UNIT history[MHP_HUMAN_HISTORY_LENGTH];
  int length;
} MHP_AGENT_HISTORY;

typedef struct STRUCT_MHP_OBJECT_MOTION_HISTORY_ARRAY {
  MHP_OBJECT_MOTION motion[MHP_OBJ_STATE_MAX][MHP_OBJECT_HISTORY_LENGTH];
  unsigned long sensor_time [MHP_OBJ_STATE_MAX][MHP_OBJECT_HISTORY_LENGTH];
  int length;
} MHP_OBJECT_MOTION_HISTORY_ARRAY;

typedef struct STRUCT_MHP_HUMAN_CONFIGURATION {
  int id;
  int changed;
  double dof[40];
} MHP_HUMAN_CONFIGURATION;


/* ---------------------------------------------- */





typedef struct STRUCT_MHP_REFERENCE_POS_POINT {
  double x;
  double y;
  double activated;
} MHP_REFERENCE_POS_POINT;

typedef enum STRUCT_MHP_HAND_CHOICE_ENUM{
  MHP_LEFT_HAND = 0,
  MHP_RIGHT_HAND = 1,
  MHP_NO_HAND = 2
} MHP_HAND_CHOICE_ENUM;

typedef struct STRUCT_MHP_HAND_CHOICE{
  MHP_HAND_CHOICE_ENUM hand;
  int unused;
} MHP_HAND_CHOICE;





typedef struct STRUCT_MHP_POS{

  /* this contains start position/orientation */
  /* for jido - a 6dof arm - you can also give a configuration */
  
  double x1,y1,z1;      /* start position */
  double dx1,dy1,dz1;  /* start orientation */
  
  /* this contains stop position/orientation */
  /* for jido - a 6dof arm - you can also give a configuration */
  
  double x2,y2,z2;     /* stop position */
  double dx2,dy2,dz2; /* stop orientation */

  double configuration; /* a flag showing if it's a configuration or not */
  double linelen; /* used only for navigation */ 
 
} MHP_POS;

typedef struct STRUCT_MHP_NAV_POS{
  MHP_3D_XYTH startpos;
  MHP_3D_XYTH goalpos;
  
  double configuration; /* a flag showing if it's a configuration or not */
  double linelen; /* used only for navigation */ 
 
} MHP_NAV_POS;

typedef struct STRUCT_MHP_PLATINE_CONF{
  double pan;
  double tilt;
} MHP_PLATINE_CONF;

typedef struct STRUCT_MHP_ROBOT_CONF{
  MHP_ROBOT_CONF_POSTERS posternames;
  //Gb_q7 armconf;
  MHP_3D_XYTH base;
  //MHP_PLATINE_CONF platine;
} MHP_ROBOT_CONF;

typedef struct STRUCT_MHP_DY_TARGET_STATE{
  GEN_STRING128 postername;  
} MHP_DY_TARGET_STATE;

typedef struct STRUCT_MHP_VOODOO_POSTER_NAMES{    
  GEN_STRING128 Limbspostername; 
  GEN_STRING128 Linkspostername;  
  GEN_STRING128 Posturepostername;
} MHP_VOODOO_POSTER_NAMES;


typedef struct STRUCT_MHP_3D_HUMAN_POSITION{
  MHP_HUMAN_POSITION position2d;
  MHP_HAND_CHOICE handchoice;
} MHP_3D_HUMAN_POSITION;

typedef struct STRUCT_MHP_DYNAMICTARGETSTATE{
  int followOnlyGest;
  int handChoice;
  double InterNodedistance;
  int stepJump;
  int firstrun;
  int plannedFrom;
  int nouse;
} MHP_DYNAMICTARGETSTATE;

/* Navigation Structures */

typedef struct STRUCT_MHP_NAV_TRAJECTORY{
  int id; /* it changes if path is changed */
  int no; /* number of points */
  double cost;
  double xcoord[MHP_NAV_MAX_TRAJ_LENTGH];
  double ycoord[MHP_NAV_MAX_TRAJ_LENTGH];
  double theta[MHP_NAV_MAX_TRAJ_LENTGH]; /* don't take into account */
} MHP_NAV_TRAJECTORY;


/* Perspective placement Structures */

typedef struct STRUCT_PSP_OBJECT_PARAMS {
  GEN_STRING128 objectName;
  double  maxRange;
  double  minRange;
} PSP_OBJECT_PARAMS;
 
typedef struct STRUCT_PSP_HUMAN_PARAMS {
  double  anglerange;
  double  maxRange;
  double  minRange;
} PSP_HUMAN_PARAMS;

typedef struct  STRUCT_PSP_GOTO_HUMAN_PARAMS {
  MHP_3D_XYTH position;
  MHP_HUMAN_STATE_ENUM state;
  int InterID;
  double Dstart;
  double Dend;
} PSP_GOTO_HUMAN_PARAMS;

typedef struct STRUCT_PSP_INTERACTION_PARAMS {
  int ID;
  int nonused;
  PSP_HUMAN_PARAMS humParams;
} PSP_INTERACTION_PARAMS;

typedef struct STRUCT_MHP_RECORD_FILE {
  char filename[64];
  int file_available;
} MHP_RECORD_FILE;


typedef enum STRUCT_MHP_PSP_TASK{
  MHP_PSP_NO_TASK,           // ordered by cost: taking less costly point first
  MHP_PSP_GIVE_TASK,        // Taking points as they are generated (non-ordered)
  MHP_PSP_PICK_TASK,             // Searching by a ramdom method
  MHP_PSP_TAKE_FROM
 }MHP_PSP_TASK;



typedef enum STRUCT_MHP_PSP_SEARCH_METHOD{
  MHP_PSP_AROUND,            // all around the human
  MHP_PSP_FRONT,             // only to search in front of the human
  MHP_PSP_AROUND_WTRAJ,
  MHP_PSP_FRONT_WTRAJ,
  MHP_PSP_AROUND_RANDOM,
  MHP_PSP_FRONT_RANDOM,
  MHP_PSP_AROUND_COMPLETE,
  MHP_PSP_FRONT_COMPLETE
 }MHP_PSP_SEARCH_METHOD;

typedef enum STRUCT_MHP_PSP_SEARCH_TYPE{
  MHP_PSP_ORDERED,           // ordered by cost: taking less costly point first
  MHP_PSP_SECUENTIAL,        // Taking points as they are generated (non-ordered)
  MHP_PSP_RANDOM,             // Searching by a ramdom method
  MHP_PSP_RANDOM_LIST
 }MHP_PSP_SEARCH_TYPE;

typedef enum STRUCT_MHP_PSP_SEARCH_GOAL{
  MHP_PSP_FFFO,              // First Found First Out (first acceptable conf. found is the one it takes)
  MHP_PSP_BCF,                // Best Cost Found (checks all points and takes best one)
  MHP_PSP_DEEP
 }MHP_PSP_SEARCH_GOAL;


typedef struct STRUCT_MHP_PSP_SEARCH_PARAMETERS {
  MHP_PSP_TASK          nextTask;
  MHP_PSP_SEARCH_METHOD methode;
  MHP_PSP_SEARCH_TYPE   stype;
  MHP_PSP_SEARCH_GOAL   goal;
}MHP_PSP_SEARCH_PARAMETERS;


typedef struct STRUCT_PSP_SEARCHBALL_PARAMS {
  MHP_3D_XYZ position;
  double  maxRange;
  double  minRange;
} PSP_SEARCHBALL_PARAMS;

#ifndef USE_HRP2_GIK
typedef enum STRUCT_MHP_Q_INDEXES{
  MHP_Q_X = 6,
  MHP_Q_Y,
  MHP_Q_Z,
  MHP_Q_RX,
  MHP_Q_RY,
  MHP_Q_RZ,
  MHP_Q_RLEG0,
  MHP_Q_RLEG1,
  MHP_Q_RLEG2,
  MHP_Q_RLEG3,
  MHP_Q_RLEG4,
  MHP_Q_RLEG5,
  MHP_Q_LLEG0,
  MHP_Q_LLEG1,
  MHP_Q_LLEG2,
  MHP_Q_LLEG3,
  MHP_Q_LLEG4,
  MHP_Q_LLEG5,
  MHP_Q_CHEST0,
  MHP_Q_CHEST1,
  MHP_Q_HEAD0,
  MHP_Q_HEAD1,
  MHP_Q_CONE,
  MHP_Q_RARM0,
  MHP_Q_RARM1,
  MHP_Q_RARM2,
  MHP_Q_RARM3,
  MHP_Q_RARM4,
  MHP_Q_RARM5,
  MHP_Q_RARM6,
  MHP_Q_RHAND0,
  MHP_Q_RHAND1,
  MHP_Q_RHAND2,
  MHP_Q_RHAND3,
  MHP_Q_RHAND4,
  MHP_Q_RHAND5,
  MHP_Q_LARM0,
  MHP_Q_LARM1,
  MHP_Q_LARM2,
  MHP_Q_LARM3,
  MHP_Q_LARM4,
  MHP_Q_LARM5,
  MHP_Q_LARM6,
  MHP_Q_LHAND0,
  MHP_Q_LHAND1,
  MHP_Q_LHAND2,
  MHP_Q_LHAND3,
  MHP_Q_LHAND4,
  MHP_Q_LHAND5
} MHP_Q_INDEXES;
#endif

typedef struct STRUCT_MHP_OBJECT_PSP {
  GEN_STRING128 name;
  int countInterest;
  int countSecuential;
  int detected;
  int notDetected;
}MHP_OBJECT_PSP;

typedef struct STRUCT_MHP_OBJECT_LIST {
  int nObjects;
  int unused;
  MHP_OBJECT_PSP object[PSP_MAX_OBJECTS];
}PSP_OBJECT_LIST;

typedef struct STRUCT_PSP_PERSPECTIVE_PARAMS {
  double perceptionThrs;
  double coneAngle;
  int secuentialMax;
  int InterestMax;
}PSP_PERSPECTIVE_PARAMS;

/* --
   -- Trajectory
   --                                                                */

/* Maximum number of reference points */
#ifdef PILO
// use pilo max
#define MHPPILO_MAX_TRAJ_SEGMENTS   PILO_MAX_TRAJ_SEGMENTS
#else
#define MHPPILO_MAX_TRAJ_SEGMENTS  100
#endif
#define MHPPILO_TRAJ_MAX_SEGMENTS MHPPILO_MAX_TRAJ_SEGMENTS

/* Local or global references */
typedef enum MHPPILO_ABS_OR_REL {
  MHPPILO_RELATIVE_TRAJ,        /* relative: init pos = current pos */
  MHPPILO_ABSOLUTE_TRAJ         /* absolute: init pos = given pos
			   -> deviation smoothed if under the given margin */
} MHPPILO_ABS_OR_REL;

/* Type of trajectory */
typedef enum MHPPILO_SEG_TYPE {
  MHPPILO_LINE_SEG,                    /* Ligne brisee (spin, trans, ...) */
  MHPPILO_ARC_SEG                      /* Succession de droites et cercles */
} MHPPILO_SEG_TYPE;

/* Header */
typedef struct MHPPILO_TRAJ_HEADER {
  int numTraj;                     /* Trajectory identification */
  int nbSegs;                      /* Number of segments */
  MHPPILO_SEG_TYPE segType;           /* broken line or arcs */
  MHPPILO_ABS_OR_REL absOrRel;         /* relative or absolute */
  double x0, y0, theta0;           /* Initial position */
  double maxInitAngleDev;          /* Max deviation on the orientation */
  double maxInitLengthDev;         /* Max deviation on the position */
} MHPPILO_TRAJ_HEADER;

/* One segment */
typedef struct MHPPILO_SEGMENT {
  double dTheta;                /* Variation angulaire (rad) */
  double dL;                    /* puis variation lineaire (m) */
  double e;                     /* Tolerance d'ecart pour le lissage (m) */
} MHPPILO_SEGMENT;

/* The trajectorory */
typedef struct MHPPILO_TRAJECTORY {
  MHPPILO_TRAJ_HEADER header;
  MHPPILO_SEGMENT seg[MHPPILO_MAX_TRAJ_SEGMENTS];
} MHPPILO_TRAJECTORY;


typedef struct STRUCT_MHP_ARM_PLAN_GOTO_Q_PARAMS {
  Gb_q7 armConfig;
  GEN_BOOL cartesian;
} STRUCT_MHP_ARM_PLAN_GOTO_Q_PARAMS;

typedef enum ENUM_MHP_MANIPULATION_TASK {
  MHP_ARM_FREE = 1, /*!< move the arm from a free configuration (in the air) to another free configuration */
  MHP_ARM_PICK_GOTO = 2,  /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support */
  MHP_ARM_TAKE_TO_FREE = 3,  /*!< move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
  MHP_ARM_TAKE_TO_PLACE = 4,  /*!< move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
  MHP_ARM_PLACE_FROM_FREE = 5,  /*!< move the arm from a free configuration to a placement configuration */
  MHP_ARM_EXTRACT = 6,  /*!< move the arm over Z axis to escape from collision */
  MHP_ARM_ESCAPE_OBJECT = 7 /*!< move the arm to escape from a placed object */
} MHP_MANIPULATION_TASK;

#ifdef JIDO
typedef struct STRUCT_MHP_ARM_PLAN_PARAMS {
  int trajId;
  GEN_BOOL startIsCurrent;
  //MANIPULATION_TASK_TYPE_STR taskType;
  MHP_MANIPULATION_TASK taskType;
  Gb_q7 armStart;
  Gb_q7 armGoto;
  GEN_STRING128 objectName;
  GEN_STRING128 supportName;
  GEN_BOOL cartesian;
  int useObjStart;
  double objStart[6];
  int useObjGoto;
  double objGoto[6];
} STRUCT_MHP_ARM_PLAN_PARAMS ;
#endif

typedef struct STRUCT_TRAJ_INFO {
   Gb_q7 armQGoto;
   Gb_v3 armXAbsGoto;
   double trajExecTime; // in second
} STRUCT_TRAJ_INFO ;

typedef struct STRUCT_GRAB_OBJECT {
  GEN_BOOL startIsCurrent;
  Gb_q7 armStart;
  GEN_STRING128 objectName;
} STRUCT_GRAB_OBJECT;

#ifdef JIDO
typedef struct STRUCT_ARM_TRAJ_ARRAY {
  //  GENMANIP_POSITION7_TRACK_STR traj[MHP_ARM_NB_MAX_TRAJ];
  SM_TRAJ_STR traj[MHP_ARM_NB_MAX_TRAJ];
} STRUCT_ARM_TRAJ_ARRAY;
#endif

/**
 * structure for passing position updates of
 * robots and humans and searching for a path then,
 * in one call to avoid collisions between several clients
 */
typedef struct STRUCT_MHP_UPD_FINDPATH {
  /**
   * human positions need not be set as long as exists = false
   */
  MHP_HUMAN_POSITION humpos1;
  MHP_HUMAN_POSITION humpos2;
  MHP_HUMAN_POSITION humpos3;
  MHP_HUMAN_POSITION humpos4;
  MHP_HUMAN_POSITION humpos5;
  MHP_NAV_POS search_definition;
} MHP_UPD_FINDPATH;

//AKP
typedef struct STRUCT_SHOW_OBJ_MIGHTABILITIES {
 int show_visible_obj;
 int show_reachable_obj;
}STRUCT_SHOW_OBJ_MIGHTABILITIES;

//AKP : For communicating with SHARY
typedef struct STRUCT_GEO_TASK_INFO {
 int plan_ID;
 char task_name[50]; // MAKE_ACCESSIBLE, SHOW, HIDE, DUMP 
 char object_name[50];
}STRUCT_GEO_TASK_INFO;

typedef struct STRUCT_GEO_ELEMENTARY_STEP_INFO {
int step_id;
int type; //1: Reaching to grasp, 2: grasping, 3: Performing the task, 4: Releasing 
int traj_id;

}STRUCT_GEO_ELEMENTARY_STEP_INFO;

typedef struct STRUCT_GEO_TASK_STATUS {
 int status;
 STRUCT_GEO_TASK_INFO geo_task_info;
 int no_of_steps;
 STRUCT_GEO_ELEMENTARY_STEP_INFO elm_step_info[10];//It will iterate from 0 to no_of_steps

}STRUCT_GEO_TASK_STATUS;

typedef struct STRUCT_GEO_PLAN_STEP_INFO {
int plan_ID;
int step_ID;
}STRUCT_GEO_PLAN_STEP_INFO;

typedef struct STRUCT_GEO_PLAN_TRAJ_INFO {
int plan_ID;
int traj_ID;
}STRUCT_GEO_PLAN_TRAJ_INFO;


#endif
