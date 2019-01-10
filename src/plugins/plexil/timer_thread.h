
/***************************************************************************
 *  timer_thread.h - timer thread
 *
 *  Created: Mon Aug 13 16:17:47 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLEXIL_TIMER_THREAD_H_
#define __PLUGINS_PLEXIL_TIMER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>

class PlexilTimerThread
	: public fawkes::Thread
{
 public:
	PlexilTimerThread();
	virtual ~PlexilTimerThread();

	virtual void loop();

	/** Callback listener pure virtual class. */
	class CallbackListener {
	public:
		/** Called for timer events. */
		virtual void timer_event() = 0;
	};

	void start_timer(CallbackListener *listener, const fawkes::Time &wait_until);
	void abort_timer();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	fawkes::Mutex *         mutex_;
	fawkes::WaitCondition * waitcond_;

	CallbackListener *      listener_;
	fawkes::Time            queued_wait_until_;
	bool                    aborted_;
};

#endif
