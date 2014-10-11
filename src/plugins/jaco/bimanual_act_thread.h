
/***************************************************************************
 *  bimanual_act_thread.h - Jaco plugin act-thread for coordinated bimanual manipulation
 *
 *  Created: Mon Sep 29 03:13:20 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_JACO_BIMANUAL_ACT_THREAD_H_
#define __PLUGINS_JACO_BIMANUAL_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
  typedef struct jaco_dual_arm_struct jaco_dual_arm_t;
}

class JacoBimanualActThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JacoBimanualActThread(fawkes::jaco_dual_arm_t *arms);
  virtual ~JacoBimanualActThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::jaco_dual_arm_t* __arms;
};


#endif
