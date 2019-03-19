
/***************************************************************************
 *  main_thread.h - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:46:37 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef _LIBS_BASEAPP_MAIN_THREAD_H_
#define _LIBS_BASEAPP_MAIN_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/mainloop/employer.h>
#include <baseapp/thread_manager.h>
#include <core/threading/thread.h>
#include <logging/multi.h>
#include <syncpoint/syncpoint_manager.h>
#include <utils/system/signal.h>

#include <getopt.h>
#include <list>
#include <string>
#include <vector>

namespace fawkes {
class Configuration;
class Configuration;
class ConfigNetworkHandler;
class NetworkLogger;
class Clock;
class TimeWait;
class AspectManager;
class PluginManager;
class Time;
class PluginNetworkHandler;
class InterruptibleBarrier;
class Barrier;
class Mutex;
class ThreadManager;
class SyncPointManager;
class FawkesNetworkManager;

class FawkesMainThread : public Thread, public MainLoopEmployer
{
public:
	FawkesMainThread(Configuration *   config,
	                 MultiLogger *     multi_logger,
	                 ThreadManager *   thread_manager,
	                 SyncPointManager *syncpoint_manager,
	                 PluginManager *   plugin_manager,
	                 const char *      load_plugins,
	                 const char *      default_plugin = 0);
	virtual ~FawkesMainThread();

	virtual void once();
	virtual void loop();

	virtual void set_mainloop_thread(Thread *mainloop_thread);

	void full_start();

	MultiLogger *logger() const;

	class Runner : public SignalHandler
	{
	public:
		Runner(FawkesMainThread *fmt, bool register_signals = true);
		~Runner();
		void run();
		void handle_signal(int signum);

	private:
		FawkesMainThread *fmt_;
		Mutex *           init_mutex_;
		bool              init_running_;
		bool              init_quit_;
		bool              sigint_running_;
		bool              register_signals_;
	};

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void destruct();

	inline void
	safe_wake(BlockedTimingAspect::WakeupHook hook, unsigned int timeout_usec)
	{
		try {
			thread_manager_->wakeup_and_wait(hook, timeout_usec);
		} catch (Exception &e) {
			if (enable_looptime_warnings_) {
				//multi_logger_->log_error("FawkesMainThread",
				//                          "Error while processing hook %s, exception follows",
				//                          BlockedTimingAspect::blocked_timing_hook_to_string(hook));
				multi_logger_->log_error("FawkesMainThread", e);
			}
		}
	}

	Configuration *config_;
	MultiLogger *  multi_logger_;
	Clock *        clock_;
	TimeWait *     time_wait_;

	Barrier *             init_barrier_;
	Thread *              mainloop_thread_;
	Mutex *               mainloop_mutex_;
	InterruptibleBarrier *mainloop_barrier_;

	char *default_plugin_;
	char *load_plugins_;

	ThreadManager *   thread_manager_;
	SyncPointManager *syncpoint_manager_;
	PluginManager *   plugin_manager_;

	std::list<std::string> recovered_threads_;
	unsigned int           desired_loop_time_usec_;
	float                  desired_loop_time_sec_;
	unsigned int           max_thread_time_usec_;
	unsigned int           max_thread_time_nanosec_;
	Time *                 loop_start_;
	Time *                 loop_end_;
	bool                   enable_looptime_warnings_;

	std::vector<RefPtr<SyncPoint>> syncpoints_start_hook_;
	std::vector<RefPtr<SyncPoint>> syncpoints_end_hook_;
};

} // end namespace fawkes

#endif
