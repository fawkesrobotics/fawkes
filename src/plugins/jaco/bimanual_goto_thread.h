
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

namespace fawkes {
  class Mutex;
}
class JacoBimanualGotoThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JacoBimanualGotoThread(fawkes::jaco_dual_arm_t *arms);
  virtual ~JacoBimanualGotoThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual bool final();

  virtual void stop();
  virtual void move_gripper(float l_f1, float l_f2, float l_f3, float r_f1, float r_f2, float r_f3);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void _lock_queues() const;
  void _unlock_queues() const;
  void _enqueue_targets(fawkes::RefPtr<fawkes::jaco_target_t> l,
                        fawkes::RefPtr<fawkes::jaco_target_t> r);

  void _move_grippers();
  void _exec_trajecs();

  void _check_final();

  typedef struct arm_struct {
    fawkes::jaco_arm_t                    *arm;
    fawkes::RefPtr<fawkes::jaco_target_t> target;
    float                                 finger_last[4]; // 3 positions + 1 counter
  } arm_struct_t;

  struct {
    arm_struct_t l;
    arm_struct_t r;
  } __arms;

  arm_struct_t* __v_arms[2]; // just a helper, to be able to iterate over both arms

  fawkes::jaco_dual_arm_t *__dual_arms; // have redundancy now, but keep this just to be sure

  fawkes::Mutex* __final_mutex;
  bool __final;
};


#endif
