
/***************************************************************************
 *  wm_thread.h - Fawkes TimeTrackerMainLoop Plugin Thread
 *
 *  Created: Fri Jun 29 11:54:58 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __PLUGINS_TTMAINLOOP_THREAD_H_
#define __PLUGINS_TTMAINLOOP_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <aspect/mainloop.h>

namespace fawkes {
  class TimeWait;
  class TimeTracker;
}

class TimeTrackerMainLoopThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::MainLoopAspect,
  public fawkes::MainLoop
{
 public:
  TimeTrackerMainLoopThread();
  virtual ~TimeTrackerMainLoopThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void mloop();

 private:
  unsigned int __loop_mod;

  fawkes::TimeWait *__time_wait;

  fawkes::TimeTracker  *__tt;
  unsigned int  __tt_loopcount;
  unsigned int  __ttc_pre_loop;
  unsigned int  __ttc_sensor;
  unsigned int  __ttc_worldstate;
  unsigned int  __ttc_think;
  unsigned int  __ttc_skill;
  unsigned int  __ttc_act;
  unsigned int  __ttc_post_loop;
  unsigned int  __ttc_netproc;
  unsigned int  __ttc_full_loop;
  unsigned int  __ttc_real_loop;

};


#endif
