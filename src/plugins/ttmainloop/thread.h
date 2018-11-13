
/***************************************************************************
 *  wm_thread.h - Fawkes TimeTrackerMainLoop Plugin Thread
 *
 *  Created: Fri Jun 29 11:54:58 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_TTMAINLOOP_THREAD_H_
#define _PLUGINS_TTMAINLOOP_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <aspect/mainloop.h>

namespace fawkes {
  class Time;
  class TimeTracker;
}

class TimeTrackerMainLoopThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::MainLoopAspect
{
 public:
  TimeTrackerMainLoopThread();
  virtual ~TimeTrackerMainLoopThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  float             output_interval_;
  fawkes::Time     *last_outp_time_;
  fawkes::Time     *now_;

  fawkes::Time                 *loop_start_;
  fawkes::Time                 *loop_end_;

  fawkes::TimeTracker  *tt_;
  unsigned int  tt_loopcount_;
  unsigned int  ttc_pre_loop_;
  unsigned int  ttc_sensor_acquire_;
  unsigned int  ttc_sensor_prepare_;
  unsigned int  ttc_sensor_process_;
  unsigned int  ttc_worldstate_;
  unsigned int  ttc_think_;
  unsigned int  ttc_skill_;
  unsigned int  ttc_act_;
  unsigned int  ttc_post_loop_;
  unsigned int  ttc_netproc_;
  unsigned int  ttc_full_loop_;
  unsigned int  ttc_real_loop_;

};


#endif
