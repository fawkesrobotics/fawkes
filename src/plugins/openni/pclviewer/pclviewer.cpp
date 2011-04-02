
/***************************************************************************
 *  pclviewer.cpp - Really simple viewer for the OpenNI PCLs
 *
 *  Created: Fri Apr 01 14:39:04 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>
#include <utils/math/angle.h>
#include <fvcams/net.h>
#include <fvcams/shmem.h>
#include <utils/system/argparser.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>

#include <blackboard/remote.h>
#include <interfaces/ObjectPositionInterface.h>

#include <GL/glut.h>
#include <cstdio>

#define GL_WIN_SIZE_X 600
#define GL_WIN_SIZE_Y 600

using namespace fawkes;
using namespace firevision;

Camera *g_pcl_cam = NULL;
Camera *g_image_cam = NULL;
unsigned char *g_rgb_buf = NULL;

GLfloat    g_scale;			/* scaling factor */
GLdouble   g_pan_x = 0.0;
GLdouble   g_pan_y = 0.0;
GLdouble   g_pan_z = 0.0;
GLint      g_mouse_state = -1;
GLint      g_mouse_button = -1;

BlackBoard *g_bb = NULL;;
ObjectPositionInterface *g_obj_if = NULL;

void
draw_points()
{
  g_pcl_cam->capture();
  if (g_image_cam) {
    g_image_cam->capture();
    convert(g_image_cam->colorspace(), RGB, g_image_cam->buffer(), g_rgb_buf,
	    g_image_cam->pixel_width(), g_image_cam->pixel_height());
    g_image_cam->dispose_buffer();
  }

  glRotatef(90., 0, 0, 1);
  glRotatef(45., 0, 1, 0);
  //glTranslatef(0, 0, -4);
  glScalef(3.0, 3.0, 3.0);
  glBegin(GL_POINTS);

  const unsigned int width  = g_pcl_cam->pixel_width();
  const unsigned int height = g_pcl_cam->pixel_height();

  float *x = (float *)g_pcl_cam->buffer();
  float *y = x + width * height;
  float *z = y + width * height;

  if (g_image_cam) {
    unsigned char *rgb = g_rgb_buf;
    //unsigned int num_values = 0, zero_values = 0;
    for (unsigned int h = 0; h < height; ++h) {
      for (unsigned int w = 0; w < width; ++w, rgb += 3) {
	//++num_values;
	if ((*x == 0) && (*y == 0) && (*z == 0)) {
	  x++; y++; z++;
	  //++zero_values;
	  continue;
	}
	
	glColor3f(rgb[0] / 255.,rgb[1] / 255.,rgb[2] / 255.);
	glVertex3f(*x++, *y++, *z++);
      }
    }
  } else {
    //unsigned int num_values = 0, zero_values = 0;
    for (unsigned int h = 0; h < height; ++h) {
      for (unsigned int w = 0; w < width; ++w) {
	//++num_values;
	if ((*x == 0) && (*y == 0) && (*z == 0)) {
	  x++; y++; z++;
	  //++zero_values;
	  continue;
	}
	
	glVertex3f(*x++, *y++, *z++);
      }
    }
  }
  //printf("Zero values %u/%u\n", zero_values, num_values);
  glEnd();

  glPointSize(5);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0, 0);
  glVertex3f(0, 0, 0);
  glColor3f(0.0, 0, 1.0);
  for (int i = -90; i <= 90; i += 10) {
    glVertex3f(cos(deg2rad(i)), sin(deg2rad(i)), 0);
  }


  glColor3f(1.0, 1.0, 1.0);
  glEnd();
  glPointSize(1);

  g_pcl_cam->dispose_buffer();
}


void
draw_object()
{
  glRotatef(90., 0, 0, 1);
  glRotatef(45., 0, 1, 0);
  //glTranslatef(0, 0, -4);
  glScalef(3.0, 3.0, 3.0);

  glPointSize(10);
  glBegin(GL_POINTS);
  glColor3f(0, 1, 0);
  glVertex4f(g_obj_if->relative_x(),
	     g_obj_if->relative_y(),
	     g_obj_if->relative_z(),
	     1.0);
  glColor3f(1.0, 1.0, 1.0);
  glEnd();
  glPointSize(1);
}


void
mouse(int button, int state, int x, int y)
{
  tbMouse(button, state, x, y);

  g_mouse_state = state;
  g_mouse_button = button;

  if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
    GLdouble model[4*4];
    GLdouble proj[4*4];
    GLint view[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, view);
    gluProject((GLdouble)x, (GLdouble)y, 0.0,
		 model, proj, view,
		 &g_pan_x, &g_pan_y, &g_pan_z);
    gluUnProject((GLdouble)x, (GLdouble)y, g_pan_z,
		 model, proj, view,
		 &g_pan_x, &g_pan_y, &g_pan_z);
    g_pan_y = -g_pan_y;
  }

  glutPostRedisplay();
}


void
motion(int x, int y)
{
  tbMotion(x, y);

  if (g_mouse_state == GLUT_DOWN && g_mouse_button == GLUT_LEFT_BUTTON) {
    GLdouble model[4*4];
    GLdouble proj[4*4];
    GLint view[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, view);
    gluProject((GLdouble)x, (GLdouble)y, 0.0,
		 model, proj, view,
		 &g_pan_x, &g_pan_y, &g_pan_z);
    gluUnProject((GLdouble)x, (GLdouble)y, g_pan_z,
		 model, proj, view,
		 &g_pan_x, &g_pan_y, &g_pan_z);
    g_pan_y = -g_pan_y;
  }

  glutPostRedisplay();
}


void
reshape(int width, int height)
{
  tbReshape(width, height);

  glViewport(0, 0, width, height);
  
  //glMatrixMode(GL_PROJECTION);
  //glLoadIdentity();
  //gluPerspective(60.0, (GLfloat)height / (GLfloat)width, 1.0, 128.0);
  //glMatrixMode(GL_MODELVIEW);
  //glLoadIdentity();
  //glTranslatef(0.0, 0.0, -3.0);
}


void
display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
    glTranslatef(g_pan_x, g_pan_y, 0.0);
    tbMatrix();

    glPushMatrix();
      draw_points();
    glPopMatrix();

    if (g_bb && g_obj_if) {
      try {
	g_obj_if->read();

	glPushMatrix();
	  draw_object();
	glPopMatrix();

	
      } catch (Exception &e) {
	printf("Interface read failed, closing");
	g_bb->close(g_obj_if);
	g_obj_if = NULL;
	delete g_bb;
	g_bb = NULL;
      }
    }

  glPopMatrix();
  glutSwapBuffers();
}


void
idle()
{
  glutPostRedisplay();
}


void
init_gl()
{
  tbInit(GLUT_MIDDLE_BUTTON);

  // Enable animation, set idle function for reset
  tbAnimate(GL_TRUE, idle);

  // Enable a single OpenGL light.
  //glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  //glEnable(GL_LIGHT0);
  //glEnable(GL_LIGHTING);

  // Use depth buffering for hidden surface elimination
  glEnable(GL_DEPTH_TEST);

  // Setup view
  glMatrixMode(GL_PROJECTION);
  gluPerspective( /* field of view in degree */ 60.0,
		  /* aspect ratio */ 1.0,
		  /* Z near */ 0.1, /* Z far */ 100.0);
  glMatrixMode(GL_MODELVIEW);
  gluLookAt(0.0, 0.0, 5.0,  // eye is at (0,0,5)
	    0.0, 0.0, 0.0,  // center is at (0,0,0)
	    0.0, 1.0, 0.0); // up is in positive Y direction
}

void
init(ArgumentParser &argp)
{
  std::string host = "localhost";
  unsigned short int port = 1910;
  if ( argp.has_arg("r") ) {
    argp.parse_hostport("r", host, port);

    printf("Connecting to %s:%u\n", host.c_str(), port);
    try {
      g_bb = new RemoteBlackBoard(host.c_str(), port);

      std::string id = "Cannikin";
      if (argp.has_arg("i")) {
	id = argp.arg("i");
      }

      g_obj_if = g_bb->open_for_reading<ObjectPositionInterface>(id.c_str());
    } catch (Exception &e) {
      e.print_trace();
      exit(-1);
    }
  }

  std::string fvhost = host;
  unsigned short int fvport = 2208;

  if (argp.has_arg("n")) {
    argp.parse_hostport("n", fvhost, fvport);
    g_pcl_cam = new NetworkCamera(fvhost.c_str(), fvport, "openni-pointcloud");
    if (argp.has_arg("R")) {
      g_image_cam = new NetworkCamera(fvhost.c_str(), fvport, "openni-image");
    }
  } else {
    g_pcl_cam = new SharedMemoryCamera("openni-pointcloud");
    if (argp.has_arg("R")) {
      g_image_cam = new SharedMemoryCamera("openni-image");
    }
  }
  g_pcl_cam->open();
  g_pcl_cam->start();
  if (argp.has_arg("R")) {
    g_image_cam->open();
    g_image_cam->start();
    g_rgb_buf = malloc_buffer(RGB, g_image_cam->pixel_width(),
			      g_image_cam->pixel_height());
  }
}


int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hr:n::si:R");
  Thread::init_main();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  glutCreateWindow("Fawkes OpenNI PCL Viewer");
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  init_gl();
  init(argp);
  glutMainLoop();
  return 0;
}
