
/***************************************************************************
 *  bimaual_goto_thread.h - Jaco plugin movement thread for coordinated bimanual manipulation
 *
 *  Created: Mon Sep 29 23:17:12 2014
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

#ifndef __PLUGINS_JACO_BIMANUAL_GOTO_THREAD_H_
#define __PLUGINS_JACO_BIMANUAL_GOTO_THREAD_H_

#include "types.h"

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

class JacoBimanualGotoThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JacoBimanualGotoThread(fawkes::jaco_arm_t *arm_l, fawkes::jaco_arm_t *arm_r);
  virtual ~JacoBimanualGotoThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual bool final();

  virtual void move_gripper(float l_f1, float l_f2, float l_f3, float r_f1, float r_f2, float r_f3);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  bool __final;

};


#endif
