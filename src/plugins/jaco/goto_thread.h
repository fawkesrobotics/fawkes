
/***************************************************************************
 *  goto_thread.h - Kinova Jaco plugin movement thread
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

#ifndef __PLUGINS_JACO_GOTO_THREAD_H_
#define __PLUGINS_JACO_GOTO_THREAD_H_

#include "types.h"

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>
#include <vector>

namespace fawkes {
  class Mutex;
}

class JacoGotoThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JacoGotoThread(const char *name, fawkes::jaco_arm_t* arm);
  virtual ~JacoGotoThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual bool final();

  virtual void set_target(float x, float y, float z, float e1, float e2, float e3, float f1=0.f, float f2=0.f, float f3=0.f);
  virtual void set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, float f1=0.f, float f2=0.f, float f3=0.f);
  virtual void move_gripper(float f1, float f2, float f3);

  virtual void pos_ready();
  virtual void pos_retract();

  virtual void stop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void _goto_target();
  void _exec_trajec(fawkes::jaco_trajec_t* trajec);

  fawkes::jaco_arm_t  *__arm;
  fawkes::Mutex       *__final_mutex;

  fawkes::RefPtr<fawkes::jaco_target_t> __target;
  float __finger_last[4]; // 3 positions + 1 counter

  bool __final;

  unsigned int __wait_status_check;

  void _check_final();
};


#endif
