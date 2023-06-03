
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
	loop_start_ = new Time(clock);
	loop_end_   = new Time(clock);

	try {
		output_interval_ = config->get_uint("/ttmainloop/output_interval");
	} catch (Exception &e) {
		output_interval_ = 5.0;
		logger->log_info(name(), "Output interval not set, using 5 seconds.");
	}

	last_outp_time_ = new Time(clock);
	now_            = new Time(clock);
	last_outp_time_->stamp();

	tt_                 = new TimeTracker("time.log");
	tt_loopcount_       = 0;
	ttc_pre_loop_       = tt_->add_class("Pre Loop");
	ttc_sensor_acquire_ = tt_->add_class("Sensor Acquire");
	ttc_sensor_prepare_ = tt_->add_class("Sensor Prepare");
	ttc_sensor_process_ = tt_->add_class("Sensor Process");
	ttc_worldstate_     = tt_->add_class("World State");
	ttc_think_          = tt_->add_class("Think");
	ttc_skill_          = tt_->add_class("Skill");
	ttc_act_            = tt_->add_class("Act");
	ttc_post_loop_      = tt_->add_class("Post Loop");
	ttc_netproc_        = tt_->add_class("Net Proc");
	ttc_full_loop_      = tt_->add_class("Full Loop");
	ttc_real_loop_      = tt_->add_class("Real Loop");
}

#define TIMETRACK_START(c1, c2, c3) \
	tt_->ping_start(c1);              \
	tt_->ping_start(c2);              \
	tt_->ping_start(c3);

#define TIMETRACK_INTER(c1, c2) \
	tt_->ping_end(c1);            \
	tt_->ping_start(c2);

#define TIMETRACK_END(c) tt_->ping_end(c);

void
TimeTrackerMainLoopThread::finalize()
{
	delete loop_start_;
	delete loop_end_;
	delete last_outp_time_;
	delete now_;
	delete tt_;
}

void
TimeTrackerMainLoopThread::loop()
{
	Thread::CancelState old_state;
	set_cancel_state(CANCEL_DISABLED, &old_state);

	TIMETRACK_START(ttc_real_loop_, ttc_full_loop_, ttc_pre_loop_);

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP);

	TIMETRACK_INTER(ttc_pre_loop_, ttc_sensor_acquire_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE);

	TIMETRACK_INTER(ttc_sensor_acquire_, ttc_sensor_prepare_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE);

	TIMETRACK_INTER(ttc_sensor_prepare_, ttc_sensor_process_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS);

	TIMETRACK_INTER(ttc_sensor_process_, ttc_worldstate_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE);

	TIMETRACK_INTER(ttc_worldstate_, ttc_think_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_THINK);

	TIMETRACK_INTER(ttc_think_, ttc_skill_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_SKILL);

	TIMETRACK_INTER(ttc_skill_, ttc_act_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_ACT);
	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC);

	TIMETRACK_INTER(ttc_act_, ttc_post_loop_)

	blocked_timing_executor->wakeup_and_wait(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP);

	TIMETRACK_INTER(ttc_post_loop_, ttc_netproc_)

	TIMETRACK_END(ttc_netproc_);
	TIMETRACK_END(ttc_real_loop_);

	set_cancel_state(old_state);

	test_cancel();

	now_->stamp();
	if ((*now_ - last_outp_time_) >= output_interval_) {
		tt_->print_to_stdout();
		tt_->print_to_file();
		tt_->reset();
		*last_outp_time_ = *now_;
	}
}
