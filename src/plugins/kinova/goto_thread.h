
/***************************************************************************
 *  goto_thread.h - Kinova plugin Jaco movement thread
 *
 *  Created: Thu Jun 20 15:04:20 2013
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

#ifndef __PLUGINS_KINOVA_KINOVA_GOTO_THREAD_H_
#define __PLUGINS_KINOVA_KINOVA_GOTO_THREAD_H_

#include "types.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace fawkes {
  class JacoArm;
  class JacoInterface;
}

class KinovaGotoThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  KinovaGotoThread();
  virtual ~KinovaGotoThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual void register_arm(fawkes::JacoArm *arm);
  virtual void unregister_arm();
  virtual void set_interface(fawkes::JacoInterface *if_jaco);

  virtual void set_target(float x, float y, float z, float e1, float e2, float e3, float f1=0.f, float f2=0.f, float f3=0.f);
  virtual void set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float f1=0.f, float f2=0.f, float f3=0.f);

  virtual void pos_ready();
  virtual void pos_retract();

  virtual void open_gripper();
  virtual void close_gripper();

  virtual void stop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::JacoArm         *__arm;
  fawkes::JacoInterface   *__if_jaco;

  float __x, __y, __z;
  float __e1, __e2, __e3;
  float __f1, __f2, __f3;
  float __joints[6];
  float __finger_last[4]; // 3 positions + 1 counter

  bool __new_target;
  fawkes::jaco_target_type_t __target_type;
  bool __final;

  unsigned int __wait_status_check;

  void check_final();
};


#endif
