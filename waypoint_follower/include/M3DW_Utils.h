/*
 * Utils.h
 *
 *  Created on: Aug 24, 2009
 *      Author: kruset
 */

#ifndef UTILS_H_
#define UTILS_H_


namespace NHPPlayerDriver
{

#ifndef DISTANCE2D
#define DISTANCE2D(x1,y1,x2,y2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))))
#endif
#ifndef DISTANCE3D
#define DISTANCE3D(x1,y1,z1,x2,y2,z2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))+((z2)-(z1))*((z2)-(z1))))
#endif

  // Normalize angle to domain -pi, pi
  #ifndef NORMALIZE
  #define NORMALIZE(z) atan2(sin(z), cos(z))
  #endif

  /**
   * The result of minmax (x, a, b) is x if x is with [a, b], a if x is left, b if x is right
   * returns
   * MINMAX (3, 2, 4) = 3, MINMAX (5, 2, 4) = 4, MINMAX (1, 2, 4 ) = 2
   */
  #define MINMAX(val, min, max) (fmax(min, fmin(max, val)))

  /**
   * The result of absmax (x, b) is x if x is with [-b, b], -b if x is left, b if x is right
   * ABSMAX (3,4) = 3, ABSMAX (5, 4) = 4, ABSMAX (-2, 3) = -2, ABSMAX (-4, 3) = -3
   */
  #define ABSMAX(val, minmax) (fmax(-minmax, fmin(minmax, val)))

  // Convert radians to degrees
  #ifndef RTOD
  #define RTOD(r) ((r) * 180 / M_PI)
  #endif

  // Convert degrees to radians
  #ifndef DTOR
  #define DTOR(d) ((d) * M_PI / 180)
  #endif

#ifndef OK
#define OK 0
#endif
#ifndef ERROR
#define ERROR (-1)
#endif

#ifndef TRUE
  #define TRUE true
#endif

#ifndef FALSE
  #define FALSE false
#endif

  typedef struct STRUCT_XYTH_COORD{
    double x;
    double y;
    double th;
  } XYTH_COORD;

  // command for a pid controller
  typedef struct STRUCT_XYA_CMD{
      double px;
      double py;
      double pa;
    } XYA_CMD;

  typedef struct STRUCT_VELOCITY {
    double heading_az;
    double speedms;
    double speedxms;
    double speedyms;

    //  double acceleration_ms;
    //  // useful for prediction
    //  double vectorx;
    //  double vectory;
  } VELOCITY;



  // Copy of player_position2d_cmd_vel_t, a command to lower level motor controllers
  typedef struct struct_position2d_cmd_vel
  {
    /** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
    XYA_CMD vel;
    /** Motor state (FALSE is either off or locked, depending on the driver). */
    int state;
  } VELOCITY_COMMAND;

long getCurrentSecs();
long getCurrentMillisecs();

}
#endif /* UTILS_H_ */
