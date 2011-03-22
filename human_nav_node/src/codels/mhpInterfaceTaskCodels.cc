#include "mhpm3d.h"
#include <display.h>
#include <mhpError.h>
#include <globalVars.h>


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

/*------------------------------------------------------------------------
 * UpdateInterface
 *
 * Description: 
 *
 * Reports:      OK
 *              S_mhp_INTERFACE_DISABLED
 */

/* mhpUpdateInterfaceMain  -  codel EXEC of UpdateInterface
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
mhpUpdateInterfaceMain(int *report)
{
//  char file[40];
//
//  if(MHP_INTERFACE_STOP_FLAG){
//    printf("Interface Stopped\n");
//    MHP_INTERFACE_STOP_FLAG = FALSE;
//    return ETHER;
//  }
//
//  SDI_F->InterfaceState->updating = GEN_TRUE;
//
//  if(SDI_F->State.planning || SDI_F->State.updating){
//    SDI_F->InterfaceState->updating = GEN_FALSE;
//    //printf("On Planning, Not refreshing\n");
//    return EXEC;
//  }
//
//  if(SDI_F->P3d.enableGraphic){
//#if !defined(SIMULATION)
//    //GENOM_UNLOCK_SDI;
//#endif
#ifdef USE_GLUT
    glutPostRedisplay ();
#else
#ifdef QT_LIBRARY
    mhpInterfaceOpenGlWidget->updateGL ();
#endif
#endif     
//    if(SDI_F->InterfaceParams.saveInterface){
//#if !defined(SIMULATION)
//      //GENOM_LOCK_SDI;
//#endif
//      //g3d_screenshot((char *)"Move3D");
//
//      // g3d_screenshot((char *)"Perspective");
//    }
//#if !defined(SIMULATION)
//    //GENOM_LOCK_SDI;
//#endif
//    SDI_F->InterfaceState->updating = GEN_FALSE;
//    return EXEC;
//  }
//  else{
//    printf("Graphics not enabled\n");
//    SDI_F->InterfaceState->updating = GEN_FALSE;
//    return ETHER;
//  }
}



/* mhpChangeCameraPosMain  -  codel EXEC of ChangeCameraPos
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
mhpChangeCameraPosMain(MHP_CAM_POS *cam_pos, int *report)
{
  G3D_Window *window = g3d_get_win_by_name((char*)"Move3D");
  
  if(!enableGraphic){
    printf("The Interface is disabled. Cannot change the camera position.\n");
    *report = S_mhp_NO_INTERFACE_LAUCHED;
    return ETHER;
  }
//  SDI_F->State.planning = TRUE;
//
//  if(SDI_F->InterfaceState->updating){
//    return EXEC;
//  }
//  else{
    if(window){
      printf("Changing camera position of the window\n");
      g3d_set_win_camera(window->vs, cam_pos->xdest, cam_pos->ydest, cam_pos->zdest,
			 cam_pos->dist, cam_pos->hrot, cam_pos->vrot, 0.0, 0.0, 1.0);
    }
//    SDI_F->State.planning = FALSE;
//  }
  return ETHER;
}


/* mhpSetInterfaceParamsMain  -  codel EXEC of SetInterfaceParams
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
mhpSetInterfaceParamsMain(MHP_INTERFACE_PARAMS *newParams, int *report)
{
  G3D_Window *win = g3d_get_win_by_name((char*)"Move3D");
  
  if(!enableGraphic){
    printf("The Interface is disabled.\n");
    *report = S_mhp_NO_INTERFACE_LAUCHED;
    return ETHER;
  }
  
  if(MHP_BTSET == NULL) {
    printf("MHP_BTSET not initialized!\n");
    *report = S_mhp_NOT_INITIALIZED;
    return ETHER;
  }
  if( (newParams->width > 0) &&  (newParams->height > 0) )
    {
#ifdef QT_LIBRARY
    mhpInterfaceOpenGlWidget->setWinSize(newParams->width, newParams->height);
#endif
#ifdef USE_GLUT
    glutReshapeWindow(newParams->width,newParams->height);
#endif
    }

  if(newParams->show_nav_obstacles) {
    hri_bt_activate(BT_OBSTACLES,MHP_BTSET);
    InterfaceState.showing_nav_obstacles = GEN_TRUE;
  }
  else {
    hri_bt_desactivate(BT_OBSTACLES,MHP_BTSET);
    InterfaceState.showing_nav_obstacles = GEN_FALSE;
  }
  if(newParams->show_nav_path) {
    hri_bt_activate(BT_PATH,MHP_BTSET);
    InterfaceState.showing_nav_path = GEN_TRUE;
  }
  else {
    hri_bt_desactivate(BT_PATH,MHP_BTSET);
    InterfaceState.showing_nav_path = GEN_FALSE;
  }
  if(newParams->show_nav_distance_grid) {
    hri_bt_activate(BT_DISTANCE,MHP_BTSET); 
    InterfaceState.showing_nav_distance_grid = GEN_TRUE;
  }
  else {
    hri_bt_desactivate(BT_DISTANCE,MHP_BTSET);
    InterfaceState.showing_nav_distance_grid = GEN_FALSE;
  }
  if(newParams->show_nav_visibility_grid) {
    hri_bt_activate(BT_VISIBILITY,MHP_BTSET);
    InterfaceState.showing_nav_visibility_grid = GEN_TRUE;
  }
  else {
    hri_bt_desactivate(BT_VISIBILITY,MHP_BTSET);
    InterfaceState.showing_nav_visibility_grid = GEN_FALSE;
  }
  if(newParams->show_nav_hidzones_grid) {
    hri_bt_activate(BT_HIDZONES,MHP_BTSET);
    InterfaceState.showing_nav_hidzones_grid = GEN_TRUE;
  }
  else {
    hri_bt_desactivate(BT_HIDZONES,MHP_BTSET);
    InterfaceState.showing_nav_hidzones_grid = GEN_FALSE;
  }
  if(!newParams->floor) {
    win->vs.displayFloor = FALSE;
    InterfaceState.showing_floor = GEN_FALSE;
  }
  else {
    win->vs.displayFloor = TRUE;
    InterfaceState.showing_floor = GEN_TRUE;
  }
  if(!newParams->walls) {
    win->vs.displayWalls = FALSE;
    InterfaceState.showing_walls = GEN_FALSE;
  }
  else {
    win->vs.displayWalls = TRUE;
    InterfaceState.showing_walls = GEN_TRUE;
  }
  
  if(!newParams->tiles) {
    win->vs.displayTiles = FALSE;
    InterfaceState.showing_tiles = GEN_FALSE;
  }
  else {
    win->vs.displayTiles = TRUE;
    InterfaceState.showing_tiles = GEN_TRUE;
  }
  
  if(!newParams->shadows) {
    win->vs.displayShadows = FALSE;
    InterfaceState.showing_shadows = GEN_FALSE;
  }
  else {
    win->vs.displayShadows = TRUE;
    InterfaceState.showing_shadows = GEN_TRUE;
  }
 
  if(newParams->draw_graph) {
    ENV.setBool(Env::drawGraph, TRUE);
    p3d_set_user_drawnjnt(newParams->draw_graph_jnt_id);
  } else {
    ENV.setBool(Env::drawGraph, FALSE);
  }
  return ETHER;
}

