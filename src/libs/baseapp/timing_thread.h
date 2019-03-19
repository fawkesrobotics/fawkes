/***************************************************************************
 *  timing_thread.h - Timing thread to achieve a desired main loop time
 *
 *  Created: Thu Jul 23 14:45:42 2015
 *  Copyright  2015-2017  Till Hofmann
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

#ifndef _LIBS_BASEAPP_TIMING_THREAD_H_
#define _LIBS_BASEAPP_TIMING_THREAD_H_

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/syncpoint_manager.h>
#include <core/threading/thread.h>
#include <utils/time/clock.h>

namespace fawkes {

class FawkesTimingThread : public Thread,
                           public SyncPointManagerAspect,
                           public ConfigurableAspect,
                           //  public ConfigurationChangeHandler,
                           public LoggingAspect
{
public:
	FawkesTimingThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	Clock *clock_;
	Time * loop_start_;
	Time * loop_end_;
	float  desired_loop_time_sec_;
	uint   desired_loop_time_usec_;
	float  min_loop_time_sec_;
	uint   min_loop_time_usec_;
	bool   enable_looptime_warnings_;

	RefPtr<SyncPoint> syncpoint_loop_start_;
	RefPtr<SyncPoint> syncpoint_loop_end_;
};

} // namespace fawkes

#endif // LIBS_BASEAPP_TIMING_THREAD_H__
