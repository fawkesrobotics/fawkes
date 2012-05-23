
/***************************************************************************
 *  joystick_thread.h - Robotino joystick thread
 *
 *  Created: Sun Nov 13 23:22:29 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_ACT_THREAD_H_
#define __PLUGINS_ROBOTINO_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <string>

namespace fawkes {
  class MotorInterface;
  class JoystickInterface;
}

class RobotinoJoystickThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  RobotinoJoystickThread();

  virtual void init();
  virtual void loop();
  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void stop();
  void send_transrot(float vx, float vy, float omega);

 private:
  fawkes::MotorInterface     *motor_if_;
  fawkes::JoystickInterface  *joystick_if_;

  unsigned int cfg_axis_forward_;
  unsigned int cfg_axis_sideward_;
  unsigned int cfg_axis_rotation_;
};


#endif
