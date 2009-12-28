
/***************************************************************************
 *  sensproc_thread.h - Laser HT sensor processing thread
 *
 *  Created: Sat Jul 04 21:34:36 2009 (RoboCup 2009, Graz)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: sensor_thread.h 2627 2009-06-25 18:08:09Z tim $
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

#ifndef __PLUGINS_LASERHT_SENSPROC_THREAD_H_
#define __PLUGINS_LASERHT_SENSPROC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

#include <vector>

namespace fawkes {
  class Laser360Interface;
  class ObjectPositionInterface;
}

class LaserHtSensorProcThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect
{
 public:
  LaserHtSensorProcThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  typedef struct {
    float angle;
    float dist;
    float x;
    float y;
  } laser_reading_t;

  void fit_line(const std::vector<laser_reading_t> &points,
		const unsigned int first_index,
		float &a, float &b, float &least_square_error);

 private:
  fawkes::Laser360Interface *__laser360_if;
  fawkes::ObjectPositionInterface *__line_if;

  float __cfg_error_threshold;
};
#endif
