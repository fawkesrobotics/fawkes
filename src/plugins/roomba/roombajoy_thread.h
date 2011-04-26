
/***************************************************************************
 *  roombajoy_thread.h - Roomba joystick control thread
 *
 *  Created: Sat Jan 29 14:34:11 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROOMBAJOY_THREAD_H_
#define __PLUGINS_ROOMBAJOY_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class Roomba500Interface;
  class JoystickInterface;
}

class RoombaJoystickThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
  RoombaJoystickThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  void stop();
  unsigned int confval(const char *path, unsigned int default_value);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::JoystickInterface  *__joy_if;
  fawkes::Roomba500Interface *__roomba500_if;

  int __last_velo;

  bool __strong_rumble;
  bool __weak_rumble;

  bool __main_brush_enabled;
  bool __side_brush_enabled;
  bool __vacuuming_enabled;

  unsigned int __cfg_but_main_brush;
  unsigned int __cfg_but_side_brush;
  unsigned int __cfg_but_vacuuming;
  unsigned int __cfg_but_dock;
  unsigned int __cfg_but_spot;
  unsigned int __cfg_but_mode;

  unsigned int __cfg_axis_forward;
  unsigned int __cfg_axis_sideward;
  unsigned int __cfg_axis_speed;

  unsigned int __cfg_min_radius;
  unsigned int __cfg_max_radius;
  unsigned int __cfg_max_velocity;
};

#endif
