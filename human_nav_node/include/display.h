/*
 * display.h
 *
 *  Created on: Mar 22, 2011
 *      Author: kruset
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#ifdef QT_LIBRARY
#include "Qt-pkg.h"
#include <QtGui/QIcon>
#include <QtGui/QPixmap>
extern QApplication* mhpInterfaceApp;
extern GLWidget* mhpInterfaceOpenGlWidget;
#endif

#ifdef USE_GLUT
#include <GL/freeglut.h>
extern GlutWindowDisplay* glutWin;
#endif

void mhp_draw_allwin_active();
void mhp_initialize_interface();

#endif /* DISPLAY_H_ */
