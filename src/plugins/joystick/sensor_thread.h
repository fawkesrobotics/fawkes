
/***************************************************************************
 *  sensor_thread.h - Joystick thread that pushes data into the interface
 *
 *  Created: Sat Nov 22 18:05:01 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_JOYSTICK_SENSOR_THREAD_H_
#define __PLUGINS_JOYSTICK_SENSOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class JoystickInterface;
}

class JoystickAcquisitionThread;

class JoystickSensorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  JoystickSensorThread(JoystickAcquisitionThread *aqt);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /** Get joystick interface.
   * @return joystick interface */
  fawkes::JoystickInterface * joystick_interface() const { return __joystick_if; }

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::JoystickInterface *__joystick_if;

  JoystickAcquisitionThread *__aqt;
};


#endif
