
/***************************************************************************
 *  openrave_thread.cpp - Kinova plugin OpenRAVE base Thread
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "openrave_base_thread.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>
 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class KinovaOpenraveBaseThread "openrave_base_thread.h"
 * Base Jaco Arm thread, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveBaseThread::KinovaOpenraveBaseThread(const char *name)
  : Thread(name, Thread::OPMODE_CONTINUOUS)
{
#ifdef HAVE_OPENRAVE
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;

  __cfg_OR_auto_load_ik = false;
#endif
}


/** Destructor. */
KinovaOpenraveBaseThread::~KinovaOpenraveBaseThread()
{
#ifdef HAVE_OPENRAVE
  __OR_env   = NULL;
  __OR_robot = NULL;
  __OR_manip = NULL;
#endif
}

void
KinovaOpenraveBaseThread::init()
{
  __target_mutex = new Mutex();
  __trajec_mutex = new Mutex();

  __target_queue = new list< vector<float> >();
  __trajec_queue = new list< vector< vector<float> >* >();

#ifdef HAVE_OPENRAVE
  __cfg_OR_use_viewer    = config->get_bool("/hardware/jaco/openrave/use_viewer");
  __cfg_OR_auto_load_ik  = config->get_bool("/hardware/jaco/openrave/auto_load_ik");

  // perform other initialization stuff (for child classes, that do not want to overload "init()")
  _init();

  // load robot
  _load_robot();

  if( __cfg_OR_use_viewer )
    openrave->start_viewer();
#endif
}

void
KinovaOpenraveBaseThread::finalize()
{
  unregister_arms();

  delete __target_mutex;
  delete __trajec_mutex;
  delete __target_queue;
  delete __trajec_queue;

#ifdef HAVE_OPENRAVE
  delete(__OR_robot);
  __OR_robot = NULL;

  delete(__OR_manip);
  __OR_manip = NULL;

  __OR_env = NULL;
#endif
}

bool
KinovaOpenraveBaseThread::trajec_ready()
{
  __trajec_mutex->lock();
  bool ready = !__trajec_queue->empty();
  __trajec_mutex->unlock();
  return ready;
}

vector< vector<float> >*
KinovaOpenraveBaseThread::pop_trajec()
{
  __trajec_mutex->lock();
  vector< vector<float> >* trajec = NULL;
  if( !__trajec_queue->empty() ) {
    trajec = __trajec_queue->front();
    __trajec_queue->pop_front();
  }
  __trajec_mutex->unlock();
  return trajec;
}
