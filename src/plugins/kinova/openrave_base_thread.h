
/***************************************************************************
 *  openrave_thread.h - Kinova plugin OpenRAVE base thread
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

#ifndef __PLUGINS_KINOVA_OPENRAVE_BASE_THREAD_H_
#define __PLUGINS_KINOVA_OPENRAVE_BASE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
#endif

#include <string>
#include <list>
#include <vector>

namespace fawkes {
  class Mutex;
  typedef struct jaco_arm_struct jaco_arm_t;
}

class KinovaOpenraveBaseThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
#ifdef HAVE_OPENRAVE
  public fawkes::OpenRaveAspect,
#endif
  public fawkes::BlackBoardAspect
{
 public:
  KinovaOpenraveBaseThread(const char *name);
  virtual ~KinovaOpenraveBaseThread();

  virtual void init();
  virtual void finalize();

  virtual void register_arm(fawkes::jaco_arm_t *arm) = 0;
  virtual void unregister_arms() = 0;

  virtual void update_openrave() = 0;

  virtual bool add_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true) = 0;
  virtual bool set_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true) = 0;

  virtual bool trajec_ready();
  virtual std::vector< std::vector<float> >* pop_trajec();

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }
  virtual void _init() {}
  virtual void _load_robot() {}

  fawkes::Mutex *__planning_mutex;
  fawkes::Mutex *__target_mutex;
  fawkes::Mutex *__trajec_mutex;

  std::list< std::vector<float> > *__target_queue;
  std::list< std::vector< std::vector<float> >* > *__trajec_queue; // list of trajectories. trajectory is a vector of configurations(=vector).

#ifdef HAVE_OPENRAVE
  fawkes::OpenRaveEnvironment* __OR_env;
  fawkes::OpenRaveRobot*       __OR_robot;
  fawkes::OpenRaveManipulator* __OR_manip;

  bool          __cfg_OR_use_viewer;
  std::string   __cfg_OR_robot_file;
  bool          __cfg_OR_auto_load_ik;
#endif
};


#endif
