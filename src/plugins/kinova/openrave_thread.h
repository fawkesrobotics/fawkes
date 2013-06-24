
/***************************************************************************
 *  openrave_thread.h - Kinova plugin OpenRAVE thread
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

#ifndef __PLUGINS_KINOVA_OPENRAVE_THREAD_H_
#define __PLUGINS_KINOVA_OPENRAVE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
#endif

#include <string>

namespace fawkes {
  class JacoArm;
  class JacoInterface;
}

class JacoOpenraveThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
#ifdef HAVE_OPENRAVE
  public fawkes::OpenRaveAspect,
#endif
  public fawkes::BlackBoardAspect
{
 public:
  JacoOpenraveThread();
  virtual ~JacoOpenraveThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual void register_arm(fawkes::JacoArm *arm);
  virtual void unregister_arm();
  virtual void set_interface(fawkes::JacoInterface *if_jaco);

  std::vector<float> set_target(float x, float y, float z, float e1, float e2, float e3);
 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::JacoArm       *__arm;
  fawkes::JacoInterface *__if_jaco;

#ifdef HAVE_OPENRAVE
  fawkes::OpenRaveEnvironment*		__OR_env;
  fawkes::OpenRaveRobot*		__OR_robot;
  fawkes::OpenRaveManipulator*		__OR_manip;

  bool          __cfg_OR_use_viewer;
  std::string   __cfg_OR_robot_file;
  bool          __cfg_OR_auto_load_ik;
#endif
};


#endif
