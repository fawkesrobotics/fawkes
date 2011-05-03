
/***************************************************************************
 *  trackball.h - Smooth mouse movements for OpenGL window
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

/* 
 *  Usage:
 *  
 *  o  call tbInit() in before any other tb call
 *  o  call tbReshape() from the reshape callback
 *  o  call tbMatrix() to get the trackball matrix rotation
 *  o  call tbStartMotion() to begin trackball movememt
 *  o  call tbStopMotion() to stop trackball movememt
 *  o  call tbMotion() from the motion callback
 *  o  call tbAnimate(GL_TRUE) if you want the trackball to continue 
 *     spinning after the mouse button has been released
 *  o  call tbAnimate(GL_FALSE) if you want the trackball to stop 
 *     spinning after the mouse button has been released
 *
 *  Typical setup:
 *
 *
    void
    init(void)
    {
      tbInit(GLUT_MIDDLE_BUTTON);
      tbAnimate(GL_TRUE);
      . . .
    }

    void
    reshape(int width, int height)
    {
      tbReshape(width, height);
      . . .
    }

    void
    display(void)
    {
      glPushMatrix();

      tbMatrix();
      . . . draw the scene . . .

      glPopMatrix();
    }

    void
    mouse(int button, int state, int x, int y)
    {
      tbMouse(button, state, x, y);
      . . .
    }

    void
    motion(int x, int y)
    {
      tbMotion(x, y);
      . . .
    }

    int
    main(int argc, char** argv)
    {
      . . .
      init();
      glutReshapeFunc(reshape);
      glutDisplayFunc(display);
      glutMouseFunc(mouse);
      glutMotionFunc(motion);
      . . .
    }
 *
 * */

#include <GL/gl.h>

/* functions */
void tbInit(GLuint button);
void tbMatrix();
void tbReshape(int width, int height);
void tbMouse(int button, int state, int x, int y);
void tbMotion(int x, int y);
void tbAnimate(GLboolean animate, void (* idle_func)() = 0);
