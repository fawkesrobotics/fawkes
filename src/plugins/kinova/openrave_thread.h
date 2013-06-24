
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
#include <plugins/openrave/aspect/openrave.h>

#include <openrave/openrave.h>

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
  public fawkes::BlackBoardAspect,
  public fawkes::OpenRaveAspect
{
 public:
  JacoOpenraveThread();
  virtual ~JacoOpenraveThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual void register_arm(fawkes::JacoArm *arm);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::JacoArm *__arm;

  fawkes::OpenRaveEnvironment*		__OR_env;
  fawkes::OpenRaveRobot*		__OR_robot;
  fawkes::OpenRaveManipulator*		__OR_manip;

  unsigned int cnt;

  fawkes::JacoInterface   *__if_jaco;
};


#endif
