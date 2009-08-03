
/***************************************************************************
 *  sensor_thread.h - Pan/tilt plugin sensor thread
 *
 *  Created: Thu Jun 18 18:58:05 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PANTILT_SENSOR_THREAD_H_
#define __PLUGINS_PANTILT_SENSOR_THREAD_H_

#include "sensor_thread.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

#include <list>

class PanTiltActThread;

class PanTiltSensorThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect
{
 public:
  PanTiltSensorThread();

  void add_act_thread(PanTiltActThread *act_thread);
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::list<PanTiltActThread *>           __act_threads;
  std::list<PanTiltActThread *>::iterator __ati;
};


#endif
