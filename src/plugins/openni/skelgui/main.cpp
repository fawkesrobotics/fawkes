
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
#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <blackboard/remote.h>
#include <blackboard/interface_observer.h>
#include <utils/system/argparser.h>

#include <map>
#include <string>
#include <queue>
#include <cstdio>
#include <GL/glut.h>

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480

using namespace fawkes;

bool g_quit = false;
bool g_pause = false;
RemoteBlackBoard *g_rbb;

UserMap  g_users;

/// @cond INTERNALS
class SkelIfObserver : public BlackBoardInterfaceObserver
{
 public:
  SkelIfObserver(BlackBoard *bb)
  {
    __queue_lock = new Mutex();
    __bb = bb;

    std::list<HumanSkeletonInterface *> skels =
      __bb->open_multiple_for_reading<HumanSkeletonInterface>("OpenNI Human *");

    std::list<HumanSkeletonProjectionInterface *> projs;

    std::list<HumanSkeletonInterface *>::iterator i;
    for (i = skels.begin(); i != skels.end(); ++i) {
      UserInfo user;
      user.skel_if = *i;
      user.proj_if =
	__bb->open_for_reading<HumanSkeletonProjectionInterface>(user.skel_if->id());

      g_users[user.skel_if->id()] = user;
    }

    bbio_add_observed_create("HumanSkeletonInterface", "OpenNI Human *");
    __bb->register_observer(this, BlackBoard::BBIO_FLAG_CREATED);
  }

  ~SkelIfObserver()
  {
    __bb->unregister_observer(this);
    delete __queue_lock;
  }

  virtual void
  bb_interface_created(const char *type, const char *id) throw()
  {
    printf("Interface %s::%s created\n", type, id);

    if (g_users.find(id) == g_users.end()) {
      __queue_lock->lock();
      __queues[__active_queue].push(id);
      __queue_lock->unlock();
    }
  }

  void
  process_queue()
  {
    __queue_lock->lock();
    unsigned int proc_queue = __active_queue;
    __active_queue = 1 - __active_queue;
    __queue_lock->unlock();
    while (! __queues[proc_queue].empty()) {
      std::string id = __queues[proc_queue].front();

      try {
	UserInfo user;
	printf("Opening HumanSkeletonInterface::%s\n", id.c_str());
	user.skel_if = __bb->open_for_reading<HumanSkeletonInterface>(id.c_str());
	try {
	  printf("Opening HumanSkeletonProjectionInterface::%s\n", id.c_str());
	  user.proj_if =
	    __bb->open_for_reading<HumanSkeletonProjectionInterface>(id.c_str());
	} catch (Exception &e) {
	  __bb->close(user.skel_if);
	  throw;
	}

	printf("Adding %s to user map\n", id.c_str());
	g_users[id] = user;
      } catch (Exception &e) {
	printf("Failed to open interfaces for '%s', exception follows", id.c_str());
	e.print_trace();
	continue;
      }

      __queues[proc_queue].pop();
    }
  }


 private:
  BlackBoard *__bb;
  Mutex *__queue_lock;
  unsigned int __active_queue;
  std::queue<std::string> __queues[2];
};
/// @endcond


SkelIfObserver *g_obs;

void clean_exit()
{
  for (UserMap::iterator i = g_users.begin(); i != g_users.end(); ++i) {
    g_rbb->close(i->second.skel_if);
    g_rbb->close(i->second.proj_if);
  }
  delete g_rbb;

  exit(1);
}

// this function is called each frame
void glut_display (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Setup the OpenGL viewpoint
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  if (! g_users.empty()) {
    if (!g_pause) {
      for (UserMap::iterator i = g_users.begin(); i != g_users.end(); ++i) {
	i->second.skel_if->read();
	i->second.proj_if->read();
      }
    }

    g_users.begin()->second.proj_if->read();
    unsigned int x_res = g_users.begin()->second.proj_if->res_x();
    unsigned int y_res = g_users.begin()->second.proj_if->res_y();
    glOrtho(0, x_res, y_res, 0, -1.0, 1.0);
    glDisable(GL_TEXTURE_2D);

    // Process the data
    draw_skeletons(g_users, x_res, y_res);
  }

  glutSwapBuffers();
}

void glut_idle (void)
{
  if (g_quit)  clean_exit();

  g_obs->process_queue();

  // Display the frame
  glutPostRedisplay();
}

void glut_keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 27:
    clean_exit();
  case 'b':
    // Draw background?
    //g_bDrawBackground = !g_bDrawBackground;
    break;
  case 'x':
    // Draw pixels at all?
    //g_bDrawPixels = !g_bDrawPixels;
    break;
  case 's':
    // Draw Skeleton?
    //g_bDrawSkeleton = !g_bDrawSkeleton;
    break;
  case 'i':
    // Print label?
    //g_bPrintID = !g_bPrintID;
    break;
  case 'l':
    // Print ID & state as label, or only ID?
    //g_bPrintState = !g_bPrintState;
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
  ArgumentParser argp(argc, argv, "hl:u:R:waLr:");

  Thread::init_main();

  std::string host = "localhost";
  unsigned short int port = 1910;
  if ( argp.has_arg("r") ) {
    argp.parse_hostport("r", host, port);
  }

  printf("Connecting to %s:%u\n", host.c_str(), port);
  g_rbb = new RemoteBlackBoard(host.c_str(), port);
  g_obs = new SkelIfObserver(g_rbb);
 
  glInit(&argc, argv);
  glutMainLoop();
}
