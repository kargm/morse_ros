/*
 * globalVars.h
 * Variables used by MHP
 *
 *  Created on: Mar 22, 2011
 *      Author: kruset
 */

#ifndef GLOBALVARS_H_
#define GLOBALVARS_H_

extern p3d_env * MHP_ENV;
extern p3d_rob * MHP_ROBOT;
extern HRI_AGENTS * MHP_AGENTS;
extern p3d_rob * MHP_VISBALL;
extern p3d_rob * trackDisc;
extern p3d_rob * pspHuman;

extern hri_gik * MHP_GIK;
extern hri_bitmapset * MHP_BTSET;
extern double grid_sampling;
extern int enableGraphic;
extern MHP_INTERFACE_STATE InterfaceState;
extern MHP_INTERFACE_PARAMS InterfaceParams;

#endif /* GLOBALVARS_H_ */
