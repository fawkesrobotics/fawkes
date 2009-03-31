
/***************************************************************************
 *  wm_thread.cpp - Fawkes TimeTrackerMainLoop Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "thread.h"

#include <core/exceptions/system.h>
#include <utils/time/tracker.h>
#include <utils/time/wait.h>

using namespace fawkes;

/** @class TimeTrackerMainLoopThread <plugins/worldmodel/wm_thread.h>
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
    __max_thread_time_usec = config->get_uint("/fawkes/mainapp/max_thread_time");
  } catch (Exception &e) {
    __max_thread_time_usec = 30000;
    logger->log_info("FawkesMainApp", "Maximum thread time not set, assuming 30ms.");
  }

  __time_wait = NULL;
  try {
    __desired_loop_time_usec = config->get_uint("/fawkes/mainapp/desired_loop_time");
    if ( __desired_loop_time_usec > 0 ) {
      __time_wait = new TimeWait(clock, __desired_loop_time_usec);
    }
  } catch (Exception &e) {
    __desired_loop_time_usec = 0;
    logger->log_info("FawkesMainApp", "Desired loop time not set, assuming 0");
  }

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
  __tt_loopcount   = 0;
  __ttc_pre_loop   = __tt->add_class("Pre Loop");
  __ttc_sensor     = __tt->add_class("Sensor");
  __ttc_worldstate = __tt->add_class("World State");
  __ttc_think      = __tt->add_class("Think");
  __ttc_skill      = __tt->add_class("Skill");
  __ttc_act        = __tt->add_class("Act");
  __ttc_post_loop  = __tt->add_class("Post Loop");
  __ttc_netproc    = __tt->add_class("Net Proc");
  __ttc_full_loop  = __tt->add_class("Full Loop");
  __ttc_real_loop  = __tt->add_class("Real Loop");
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
  delete __time_wait;
  delete __tt;
}

void
TimeTrackerMainLoopThread::loop()
{
  if ( ! blocked_timing_executor->timed_threads_exist() ) {
    logger->log_debug("TimeTrackerMainLoopThread", "No threads exist, waiting");
    try {
      blocked_timing_executor->wait_for_timed_threads();
      logger->log_debug("TimeTrackerMainLoopThread", "Timed threads have been "
			"added, running main loop now");
    } catch (InterruptedException &e) {
      logger->log_debug("TimeTrackerMainLoopThread", "Waiting for timed threads interrupted");
      return;
    }
  }

  TIMETRACK_START(__ttc_real_loop, __ttc_full_loop, __ttc_pre_loop);

  if ( __time_wait ) {
    __time_wait->mark_start();
  }
  __loop_start->stamp_systime();

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP );

  TIMETRACK_INTER(__ttc_pre_loop, __ttc_sensor)

  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR );
  blocked_timing_executor->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS );

  TIMETRACK_INTER(__ttc_sensor, __ttc_worldstate)

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

  test_cancel();

  if ( __time_wait ) {
    __time_wait->wait_systime();
  } else {
    yield();
  }

  TIMETRACK_END(__ttc_full_loop);

  __now->stamp();
  if ( (*__now - __last_outp_time) >= __output_interval ) {
    __tt->print_to_stdout();
    __tt->print_to_file();
    __tt->reset();
    *__last_outp_time = *__now;
  }

  try {
    blocked_timing_executor->try_recover(__recovered_threads);
    if ( ! __recovered_threads.empty() ) {
      // threads have been recovered!
      std::string s;
      if ( __recovered_threads.size() == 1 ) {
	s = std::string("The thread ") + __recovered_threads.front() +
	  " could be recovered and resumes normal operation";
      } else {
	s = "The following threads could be recovered and resumed normal operation: ";
	for (std::list<std::string>::iterator i = __recovered_threads.begin();
	     i != __recovered_threads.end(); ++i) {
	  s += *i + " ";
	}
      }
      __recovered_threads.clear();
      logger->log_warn("FawkesMainThread", "%s", s.c_str());
    }

    if (__desired_loop_time_sec > 0) {
      __loop_end->stamp_systime();
      float loop_time = *__loop_end - __loop_start;
      if (loop_time > __desired_loop_time_sec) {
	logger->log_warn("FawkesMainThread", "Loop time exceeded, "
				 "desired: %f sec (%u usec),  actual: %f sec",
				 __desired_loop_time_sec, __desired_loop_time_usec,
				 loop_time);
      }
    }
    if ( __time_wait ) {
      __time_wait->wait_systime();
    } else {
      yield();
    }
  } catch (Exception &e) {
    logger->log_warn("FawkesMainThread",
			     "Exception caught while executing default main "
			     "loop, ignoring.");
    logger->log_warn("FawkesMainThread", e);
  } catch (std::exception &e) {
    logger->log_warn("FawkesMainThread",
			     "STL Exception caught while executing default main "
			     "loop, ignoring. (what: %s)", e.what());
  }
  // catch ... is not a good idea, would catch cancellation exception
  // at least needs to be rethrown.
}
