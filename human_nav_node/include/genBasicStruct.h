// TODO: use genBasic itself

#ifndef GENBASIC_STRUCT_H
#define GENBASIC_STRUCT_H


/* ------------------------------------------------------------
 * USEFULL
 */
typedef enum GEN_BOOL {
  GEN_TRUE=1,
  GEN_FALSE=0
} GEN_BOOL;


/* ------------------------------------------------------------
 * STRINGS
 */
typedef struct GEN_STRING16 {
  char name[16];
} GEN_STRING16;

typedef struct GEN_STRING32 {
  char name[32];
} GEN_STRING32;

typedef struct GEN_STRING64 {
  char name[64];
} GEN_STRING64;

typedef struct GEN_STRING128 {
  char name[128];
} GEN_STRING128;

typedef struct GEN_STRING256 {
  char name[256];
} GEN_STRING256;

/* ------------------------------------------------------------
 * GEOM CARTESIAN
 */
typedef struct GEN_POINT_2D {
  double x;
  double y;
} GEN_POINT_2D;

typedef struct GEN_CONFIG_2D {
  double x;
  double y;
  double theta;
} GEN_CONFIG_2D;

typedef struct GEN_POINT_3D {
  double x;
  double y;
  double z;
} GEN_POINT_3D;


typedef struct GEN_SEG_2D {
  GEN_POINT_2D pt1;
  GEN_POINT_2D pt2;
} GEN_SEG_2D;

typedef struct GEN_SEG_3D {
  GEN_POINT_3D pt1;
  GEN_POINT_3D pt2;
} GEN_SEG_3D;


/* ------------------------------------------------------------
 * GEOM POLAR
 */
typedef struct GEN_POLAR_2D {
  double rho;
  double theta;
} GEN_POLAR_2D;

#endif /* GENBASIC_STRUCT_H */

