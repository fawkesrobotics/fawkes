
/***************************************************************************
 *  act_thread.h - Kinova Jaco plugin act-thread
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

#ifndef __PLUGINS_JACO_ACT_THREAD_H_
#define __PLUGINS_JACO_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
  typedef struct jaco_arm_struct jaco_arm_t;
}

class JacoActThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JacoActThread(const char *name, fawkes::jaco_arm_t* arm);
  virtual ~JacoActThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void _initialize();
  bool _is_initializing();
  void _process_msgs();

  fawkes::jaco_arm_t* __arm;

  bool __cfg_auto_init;
  bool __cfg_auto_calib;
};


#endif
