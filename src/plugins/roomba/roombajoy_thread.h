
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

#ifndef _PLUGINS_ROOMBAJOY_THREAD_H_
#define _PLUGINS_ROOMBAJOY_THREAD_H_

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
  fawkes::JoystickInterface  *joy_if_;
  fawkes::Roomba500Interface *roomba500_if_;

  int last_velo_;

  bool strong_rumble_;
  bool weak_rumble_;

  bool main_brush_enabled_;
  bool side_brush_enabled_;
  bool vacuuming_enabled_;

  unsigned int cfg_but_main_brush_;
  unsigned int cfg_but_side_brush_;
  unsigned int cfg_but_vacuuming_;
  unsigned int cfg_but_dock_;
  unsigned int cfg_but_spot_;
  unsigned int cfg_but_mode_;

  unsigned int cfg_axis_forward_;
  unsigned int cfg_axis_sideward_;
  unsigned int cfg_axis_speed_;

  unsigned int cfg_min_radius_;
  unsigned int cfg_max_radius_;
  unsigned int cfg_max_velocity_;
};

#endif
