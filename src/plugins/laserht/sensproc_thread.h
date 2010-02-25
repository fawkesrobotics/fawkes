
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
#include <string>

namespace fawkes {
  class Laser360Interface;
  class ObjectPositionInterface;
  class VisualDisplay2DInterface;
  class TimeTracker;
}
class HoughTransform;

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

  void line_points_from_params(float r, float phi,
			       float &x1, float &y1, float &x2, float &y2);

 private:
  fawkes::Laser360Interface *__laser360_if;
  fawkes::ObjectPositionInterface *__line_if;
  fawkes::VisualDisplay2DInterface *__visdisp_if;

  unsigned int __cfg_num_samples;
  unsigned int __cfg_vote_threshold;
  float        __cfg_r_scale;
  std::string  __cfg_laser_ifid;
  bool         __cfg_enable_disp;
  float        __cfg_fitting_error_threshold;
  float        __cfg_dist_threshold;
  

  HoughTransform *__ht;
  unsigned int    __num_vals;
  int           **__values;
  float           __angle_step;
  float           __r_scale;

#ifdef LASERHT_TIMETRACKER
  fawkes::TimeTracker *__tt;
  unsigned int __tt_loop;
  unsigned int __ttc_reset;
  unsigned int __ttc_process;
  unsigned int __ttc_fitting;
  unsigned int __ttc_total;
#endif
};
#endif
