
/***************************************************************************
 *  thread.cpp - Fawkes TimeTrackerMainLoop Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
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

#include "thread.h"

#include <core/exceptions/system.h>
#include <utils/time/tracker.h>

using namespace fawkes;

/** @class TimeTrackerMainLoopThread <plugins/ttmainloop/thread.h>
 * Main thread of time tracker main loop plugin.
 * @author Tim Niemueller
 */

/** Constructor. */
TimeTrackerMainLoopThread::TimeTrackerMainLoopThread()
  : Thread("TimeTrackerMainLoopThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
TimeTrackerMainLoopThread::~TimeTrackerMainLoopThread()
{
}


void
TimeTrackerMainLoopThread::init()
{
  __loop_start = new Time(clock);
  __loop_end   = new Time(clock);

  try {
    __output_interval = config->get_uint("/ttmainloop/output_interval");
  } catch (Exception &e) {
    __output_interval = 5.0;
    logger->log_info(name(), "Output interval not set, using 5 seconds.");
  }


  __last_outp_time = new Time(clock);
  __now            = new Time(clock);
  __last_outp_time->stamp();

  __tt = new TimeTracker("time.log");
  __tt_loopcount        = 0;
  __ttc_pre_loop        = __tt->add_class("Pre Loop");
  __ttc_sensor_acquire  = __tt->add_class("Sensor Acquire");
  __ttc_sensor_prepare  = __tt->add_class("Sensor Prepare");
  __ttc_sensor_process  = __tt->add_class("Sensor Process");
  __ttc_worldstate      = __tt->add_class("World State");
  __ttc_think           = __tt->add_class("Think");
  __ttc_skill           = __tt->add_class("Skill");
  __ttc_act             = __tt->add_class("Act");
  __ttc_post_loop       = __tt->add_class("Post Loop");
  __ttc_netproc         = __tt->add_class("Net Proc");
  __ttc_full_loop       = __tt->add_class("Full Loop");
  __ttc_real_loop       = __tt->add_class("Real Loop");
}


#define TIMETRACK_START(c1, c2, c3)		\
  __tt->ping_start(c1);				\
  __tt->ping_start(c2);				\
  __tt->ping_start(c3);

#define TIMETRACK_INTER(c1, c2)			\
 __tt->ping_end(c1);				\
 __tt->ping_start(c2);

#define TIMETRACK_END(c)			\
  __tt->ping_end(c);

void
TimeTrackerMainLoopThread::finalize()
{
  delete __tt;
}

void
TimeTrackerMainLoopThread::loop()
{
  Thread::CancelState old_state;
  set_cancel_state(CANCEL_DISABLED, &old_state);

  TIMETRACK_START(__ttc_real_loop, __ttc_full_loop, __ttc_pre_loop);

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP );

  TIMETRACK_INTER(__ttc_pre_loop, __ttc_sensor_acquire)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE );

  TIMETRACK_INTER(__ttc_sensor_acquire, __ttc_sensor_prepare)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE );

  TIMETRACK_INTER(__ttc_sensor_prepare, __ttc_sensor_process)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS );

  TIMETRACK_INTER(__ttc_sensor_process, __ttc_worldstate)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE );

  TIMETRACK_INTER(__ttc_worldstate, __ttc_think)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_THINK );

  TIMETRACK_INTER(__ttc_think, __ttc_skill)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SKILL );

  TIMETRACK_INTER(__ttc_skill, __ttc_act)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT );
  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC );

  TIMETRACK_INTER(__ttc_act, __ttc_post_loop)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP );

  TIMETRACK_INTER(__ttc_post_loop, __ttc_netproc)

  TIMETRACK_END(__ttc_netproc);
  TIMETRACK_END(__ttc_real_loop);

  set_cancel_state(old_state);

  test_cancel();

  __now->stamp();
  if ( (*__now - __last_outp_time) >= __output_interval ) {
    __tt->print_to_stdout();
    __tt->print_to_file();
    __tt->reset();
    *__last_outp_time = *__now;
  }

}
