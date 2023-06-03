
/***************************************************************************
 *  timesync_thread.h - Thread to synchronize loops, PRE_LOOP hook
 *
 *  Created: Tue Sep 30 14:29:12 2008
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

#ifndef _PLUGINS_PLAYER_TIMESYNC_THREAD_H_
#define _PLUGINS_PLAYER_TIMESYNC_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/time_source.h>
#include <core/threading/thread.h>
#include <utils/time/simts.h>

class PlayerTimeSyncThread : public fawkes::Thread, public fawkes::BlockedTimingAspect
//public fawkes::TimeSourceAspect
{
public:
	PlayerTimeSyncThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::SimulatorTimeSource simts_;
};

#endif
