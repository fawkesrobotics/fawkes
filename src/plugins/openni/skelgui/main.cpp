
/***************************************************************************
 *  main.cpp - Skeleton Visualization GUI: main program
 *
 *  Created: Wed Mar 02 11:15:53 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "skel_drawer.h"
#include "image_drawer.h"
#include "depth_drawer.h"
#include <plugins/openni/utils/skel_if_observer.h>
#include <plugins/openni/utils/hand_if_observer.h>

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <blackboard/remote.h>
#include <blackboard/interface_observer.h>
#include <utils/system/argparser.h>
#include <fvcams/net.h>
#include <fvcams/shmem.h>

#include <map>
#include <string>
#include <queue>
#include <cstdio>
#include <cstring>
#include <GL/glut.h>

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480

using namespace fawkes;
using namespace fawkes::openni;
using namespace firevision;

bool g_quit = false;
bool g_pause = false;
bool g_draw_image = true;
bool g_draw_depth = false;
bool g_draw_skeleton = true;
RemoteBlackBoard *g_rbb;
Camera *g_image_cam;
Camera *g_depth_cam;
Camera *g_label_cam;
SkelGuiImageDrawer *g_image_drawer = NULL;
SkelGuiDepthDrawer *g_depth_drawer = NULL;
SkelGuiSkeletonDrawer *g_skeleton_drawer = NULL;

UserMap  g_users;
HandMap  g_hands;

SkelIfObserver *g_obs;
HandIfObserver *g_hands_obs;

void clean_exit()
{
  delete g_obs;
  delete g_hands_obs;
  for (UserMap::iterator i = g_users.begin(); i != g_users.end(); ++i) {
    g_rbb->close(i->second.skel_if);
    g_rbb->close(i->second.proj_if);
  }
  delete g_rbb;

  delete g_image_cam;
  delete g_depth_cam;
  delete g_label_cam;
  delete g_image_drawer;
  delete g_depth_drawer;
  delete g_skeleton_drawer;

  exit(1);
}


// this function is called each frame
void glut_display (void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Setup the OpenGL viewpoint
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

  if (g_draw_image && g_image_drawer)  g_image_drawer->draw();
  if (g_draw_depth && g_depth_drawer)  g_depth_drawer->draw();

  if (! g_users.empty() || ! g_hands.empty() ) {
    if (!g_pause) {
      for (UserMap::iterator i = g_users.begin(); i != g_users.end(); ++i) {
	i->second.skel_if->read();
	i->second.proj_if->read();
      }
      for (HandMap::iterator i = g_hands.begin(); i != g_hands.end(); ++i) {
	i->second.hand_if->read();
      }
    }

    // Process the data
    if (g_draw_skeleton)  g_skeleton_drawer->draw();
  }

  glutSwapBuffers();
}


void glut_idle()
{
  if (g_quit)  clean_exit();

  g_obs->process_queue();
  g_hands_obs->process_queue();

  // Display the frame
  glutPostRedisplay();
}

void
glut_keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 27:
  case 'q':
    clean_exit();
  case 'i':
    g_draw_image = ! g_draw_image;
    if (g_draw_image)  g_draw_depth = false;
    break;
  case 'd':
    g_draw_depth = ! g_draw_depth;
    if (g_draw_depth)  g_draw_image = false;
    break;
  case 's':
    g_draw_skeleton = ! g_draw_skeleton;
    break;
  case 'l':
    g_skeleton_drawer->toggle_print_state();
    break;
  case 'L':
    if (g_depth_drawer) g_depth_drawer->toggle_show_labels();
    break;
  case'p':
    g_pause = !g_pause;
    break;
  }
}

void glInit (int * pargc, char ** argv)
{
  glutInit(pargc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
  glutCreateWindow ("Fawkes Skeleton GUI");
  //glutFullScreen();
  glutSetCursor(GLUT_CURSOR_NONE);

  glutKeyboardFunc(glut_keyboard);
  glutDisplayFunc(glut_display);
  glutIdleFunc(glut_idle);

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);

  glEnableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);
}

int main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hr:n::js");

  Thread::init_main();
  glInit(&argc, argv);

  std::string host = "localhost";
  unsigned short int port = 1910;
  if ( argp.has_arg("r") ) {
    argp.parse_hostport("r", host, port);
  }

  std::string fvhost = host;
  unsigned short int fvport = 2208;
  if ( argp.has_arg("n") ) {
    argp.parse_hostport("n", fvhost, fvport);
  }

  printf("Connecting to %s:%u\n", host.c_str(), port);
  try {
    g_rbb = new RemoteBlackBoard(host.c_str(), port);
  } catch (Exception &e) {
    e.print_trace();
    exit(-1);
  }
  g_obs = new SkelIfObserver(g_rbb, g_users);
  g_hands_obs = new HandIfObserver(g_rbb, g_hands);

  g_image_cam = NULL;
  g_depth_cam = NULL;
  g_label_cam = NULL;
  g_image_drawer = NULL;

  if (argp.has_arg("n") || argp.has_arg("s")) {
    if (argp.has_arg("n")) {
      g_image_cam = new NetworkCamera(fvhost.c_str(), fvport, "openni-image",
				      argp.has_arg("j"));
      g_depth_cam = new NetworkCamera(fvhost.c_str(), fvport, "openni-depth");
      g_label_cam = new NetworkCamera(fvhost.c_str(), fvport, "openni-labels");
    } else {
      g_image_cam = new SharedMemoryCamera("openni-image");
      g_depth_cam = new SharedMemoryCamera("openni-depth");
      try {
	g_label_cam = new SharedMemoryCamera("openni-labels");
	g_label_cam->open();
	g_label_cam->start();
	if ((g_label_cam->pixel_width() != GL_WIN_SIZE_X) ||
	    (g_label_cam->pixel_height() != GL_WIN_SIZE_Y)) {
	  delete g_label_cam;
	  g_label_cam = NULL;
	  throw Exception("Invalid label cam");
	}
      } catch (Exception &e) {
	printf("Cannot open label cam, user tracker not running? "
	       "Disabling labels.\n");
      }
    }
    g_image_cam->open();
    g_image_cam->start();
    g_depth_cam->open();
    g_depth_cam->start();
    if (g_label_cam) {
      g_label_cam->open();
      g_label_cam->start();
    }

    if ((g_image_cam->pixel_width() != GL_WIN_SIZE_X) ||
	(g_image_cam->pixel_height() != GL_WIN_SIZE_Y) ||
	(g_depth_cam->pixel_width() != GL_WIN_SIZE_X) ||
	(g_depth_cam->pixel_height() != GL_WIN_SIZE_Y) )
    {
      printf("Image size different from window size, closing camera");
      delete g_image_cam;
      delete g_depth_cam;
      g_image_cam = g_depth_cam = NULL;
    }

    g_image_drawer = new SkelGuiImageDrawer(g_image_cam);
    g_depth_drawer = new SkelGuiDepthDrawer(g_depth_cam, g_label_cam, 10000);
  }

  g_skeleton_drawer = new SkelGuiSkeletonDrawer(g_users, g_hands);

  glutMainLoop();
}
