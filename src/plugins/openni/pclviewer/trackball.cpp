
/***************************************************************************
 *  trackball.cpp - Smooth mouse movements for OpenGL window
 *
 *  Created: Fri Apr 01 19:56:31 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *
 *  The code has is based on the OpenGL example "smooth" by Nate Robins
 *  It states:
 *  "Simple trackball-like motion adapted (ripped off) from projtex.c
 *   (written by David Yu and David Blythe).  See the SIGGRAPH '96
 *   Advanced OpenGL course notes."
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "trackball.h"

#include <GL/glut.h>
#include <cmath>


// globals
static GLuint    tb_lasttime;
static GLfloat   tb_lastposition[3];

static GLfloat   tb_angle = 0.0;
static GLfloat   tb_axis[3];
static GLfloat   tb_transform[4][4];

static GLuint    tb_width;
static GLuint    tb_height;

static GLint     tb_button = -1;
static GLboolean tb_tracking = GL_FALSE;
static GLboolean tb_animate = GL_TRUE;

static void (*   tb_original_idle_func)();

// functions
static void
_tbPointToVector(int x, int y, int width, int height, float v[3])
{
  float d, a;

  // project x, y onto a hemi-sphere centered within width, height.
  v[0] = (2.0 * x - width) / width;
  v[1] = (height - 2.0 * y) / height;
  d = sqrt(v[0] * v[0] + v[1] * v[1]);
  v[2] = cos((3.14159265 / 2.0) * ((d < 1.0) ? d : 1.0));
  a = 1.0 / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] *= a;
  v[1] *= a;
  v[2] *= a;
}

static void
_tbAnimate(void)
{
  tb_original_idle_func();
  glutPostRedisplay();
}

void
_tbStartMotion(int x, int y, int button, int time)
{
  if (tb_button == -1) return;

  tb_tracking = GL_TRUE;
  tb_lasttime = time;
  _tbPointToVector(x, y, tb_width, tb_height, tb_lastposition);
}

void
_tbStopMotion(int button, unsigned time)
{
  if (tb_button == -1) return;

  tb_tracking = GL_FALSE;

  if (time == tb_lasttime && tb_animate) {
    glutIdleFunc(_tbAnimate);
  } else {
    tb_angle = 0.0;
    if (tb_animate)
      glutIdleFunc(tb_original_idle_func);
  }
}

void
tbAnimate(GLboolean animate, void (* idle_func)())
{
  tb_animate = animate;
  tb_original_idle_func = idle_func;
}

void
tbInit(GLuint button)
{
  tb_button = button;
  tb_angle = 0.0;

  // put the identity in the trackball transform
  glPushMatrix();
  glLoadIdentity();
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();
}

void
tbMatrix()
{
  if (tb_button == -1) return;

  glPushMatrix();
  glLoadIdentity();
  glRotatef(tb_angle, tb_axis[0], tb_axis[1], tb_axis[2]);
  glMultMatrixf((GLfloat *)tb_transform);
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();

  glMultMatrixf((GLfloat *)tb_transform);
}

void
tbReshape(int width, int height)
{
  if (tb_button == -1) return;

  tb_width  = width;
  tb_height = height;
}

void
tbMouse(int button, int state, int x, int y)
{
  if (tb_button == -1) return;

  if (state == GLUT_DOWN && button == tb_button)
    _tbStartMotion(x, y, button, glutGet(GLUT_ELAPSED_TIME));
  else if (state == GLUT_UP && button == tb_button)
    _tbStopMotion(button, glutGet(GLUT_ELAPSED_TIME));
}

void
tbMotion(int x, int y)
{
  GLfloat current_position[3], dx, dy, dz;

  if (tb_button == -1) return;

  if (tb_tracking == GL_FALSE)
    return;

  _tbPointToVector(x, y, tb_width, tb_height, current_position);

  /* calculate the angle to rotate by (directly proportional to the
     length of the mouse movement */
  dx = current_position[0] - tb_lastposition[0];
  dy = current_position[1] - tb_lastposition[1];
  dz = current_position[2] - tb_lastposition[2];
  tb_angle = 90.0 * sqrt(dx * dx + dy * dy + dz * dz);

  // calculate the axis of rotation (cross product)
  tb_axis[0] = tb_lastposition[1] * current_position[2] - 
               tb_lastposition[2] * current_position[1];
  tb_axis[1] = tb_lastposition[2] * current_position[0] - 
               tb_lastposition[0] * current_position[2];
  tb_axis[2] = tb_lastposition[0] * current_position[1] - 
               tb_lastposition[1] * current_position[0];

  // reset for next time
  tb_lasttime = glutGet(GLUT_ELAPSED_TIME);
  tb_lastposition[0] = current_position[0];
  tb_lastposition[1] = current_position[1];
  tb_lastposition[2] = current_position[2];

  // remember to draw new position
  glutPostRedisplay();
}
