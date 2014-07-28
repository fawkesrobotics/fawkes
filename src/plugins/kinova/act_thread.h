
/***************************************************************************
 *  act_thread.h - Kinova plugin Jaco thread
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

#ifndef __PLUGINS_KINOVA_ACT_THREAD_H_
#define __PLUGINS_KINOVA_ACT_THREAD_H_

#include "info_thread.h"
#include "goto_thread.h"
#include "openrave_thread.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace KinDrv {
  class JacoArm;
}

namespace fawkes {
  typedef struct jaco_arm_struct jaco_arm_t;
  typedef struct jaco_dual_arm_struct jaco_dual_arm_t;
}

class KinovaActThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  KinovaActThread(KinovaInfoThread *info_thread, KinovaGotoThread *goto_thread, JacoOpenraveThread *openrave_thread);
  virtual ~KinovaActThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void (KinovaActThread::*_submit_iface_changes)();
  bool (KinovaActThread::*_is_initializing)();
  void (KinovaActThread::*_process_msgs)();

  void _initialize_single();
  void _initialize_dual();
  bool _is_initializing_single();
  bool _is_initializing_dual();
  void _submit_iface_single();
  void _submit_iface_dual();
  void _process_msgs_single();
  void _process_msgs_dual();

  fawkes::jaco_arm_t      __arm;
  fawkes::jaco_dual_arm_t __dual_arm;

  bool __cfg_auto_init;
  bool __cfg_auto_calib;
  bool __cfg_is_dual_arm;

  KinovaInfoThread   *__info_thread;
  KinovaGotoThread   *__goto_thread;
  JacoOpenraveThread *__openrave_thread;
};


#endif
