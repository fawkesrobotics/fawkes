
/***************************************************************************
 *  main_thread.cpp - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:47:50 2006
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <aspect/manager.h>
#include <baseapp/main_thread.h>
#include <config/config.h>
#include <core/exceptions/system.h>
#include <core/macros.h>
#include <core/threading/interruptible_barrier.h>
#include <core/threading/mutex_locker.h>
#include <core/version.h>
#include <plugin/loader.h>
#include <plugin/manager.h>
#include <utils/time/clock.h>
#include <utils/time/wait.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

namespace fawkes {

/** @class FawkesMainThread <baseapp/main_thread.h>
 * Fawkes default main thread.
 * This thread initializes all important stuff like the BlackBoard,
 * handles plugins and wakes up threads at defined hooks.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration to use
 * @param multi_logger basic multi logger to use, a network logger will be
 * added in the ctor.
 * @param thread_manager thread manager used to wakeup threads
 * @param syncpoint_manager syncpoint manager used to manage syncpoints
 * @param plugin_manager plugin manager to load the desired plugins
 * @param load_plugins string with comma-separated list of names of plugins
 * to load on startup.
 * @param default_plugin additional default plugin name
 */
FawkesMainThread::FawkesMainThread(Configuration *   config,
                                   MultiLogger *     multi_logger,
                                   ThreadManager *   thread_manager,
                                   SyncPointManager *syncpoint_manager,
                                   PluginManager *   plugin_manager,
                                   const char *      load_plugins,
                                   const char *      default_plugin)
: Thread("FawkesMainThread")
{
	plugin_manager_    = plugin_manager;
	thread_manager_    = thread_manager;
	syncpoint_manager_ = syncpoint_manager;
	multi_logger_      = multi_logger;
	config_            = config;

	mainloop_thread_  = NULL;
	mainloop_mutex_   = new Mutex();
	mainloop_barrier_ = new InterruptibleBarrier(mainloop_mutex_, 2);

	load_plugins_ = NULL;
	if (load_plugins) {
		load_plugins_ = strdup(load_plugins);
	}

	default_plugin_ = NULL;
	if (default_plugin) {
		default_plugin_ = strdup(default_plugin);
	}

	/* Clock */
	clock_ = Clock::instance();

	loop_start_ = new Time(clock_);
	loop_end_   = new Time(clock_);
	try {
		max_thread_time_usec_ = config_->get_uint("/fawkes/mainapp/max_thread_time");
	} catch (Exception &e) {
		max_thread_time_usec_ = 30000;
		multi_logger_->log_info("FawkesMainApp", "Maximum thread time not set, assuming 30ms.");
	}
	max_thread_time_nanosec_ = max_thread_time_usec_ * 1000;

	time_wait_ = NULL;
	try {
		desired_loop_time_usec_ = config_->get_uint("/fawkes/mainapp/desired_loop_time");
		if (desired_loop_time_usec_ > 0) {
			time_wait_ = new TimeWait(clock_, desired_loop_time_usec_);
		}
	} catch (Exception &e) {
		desired_loop_time_usec_ = 0;
		multi_logger_->log_info("FawkesMainApp", "Desired loop time not set, assuming 0");
	}

	desired_loop_time_sec_ = (float)desired_loop_time_usec_ / 1000000.f;

	try {
		enable_looptime_warnings_ = config_->get_bool("/fawkes/mainapp/enable_looptime_warnings");
		if (!enable_looptime_warnings_) {
			multi_logger_->log_debug(name(), "loop time warnings are disabled");
		}
	} catch (Exception &e) {
		enable_looptime_warnings_ = true;
	}
}

/** Destructor. */
FawkesMainThread::~FawkesMainThread()
{
	destruct();
}

/** Destruct.
 * Mimics destructor, but may be called in ctor exceptions.
 */
void
FawkesMainThread::destruct()
{
	try {
		config_->try_dump();
	} catch (CouldNotOpenFileException &e) {
		if (e.get_errno() == EACCES) {
			multi_logger_->log_warn("FawkesMainThread",
			                        "Cannot write to dump file, "
			                        "no write ");
			multi_logger_->log_warn("FawkesMainThread",
			                        "permission for file or "
			                        "directory. This");
			multi_logger_->log_warn("FawkesMainThread",
			                        "usually happens if running "
			                        "with system-wide");
			multi_logger_->log_warn("FawkesMainThread",
			                        "installed Fawkes as non-root "
			                        "user. Make");
			multi_logger_->log_warn("FawkesMainThread",
			                        "configuration changes to the "
			                        "host-based");
			multi_logger_->log_warn("FawkesMainThread",
			                        "database (set as non-default "
			                        "values).");
		} else {
			multi_logger_->log_warn("FawkesMainThread",
			                        "Failed to dump default "
			                        "config (open), exception follows.");
			multi_logger_->log_warn("FawkesMainThread", e);
		}
	} catch (Exception &e) {
		multi_logger_->log_warn("FawkesMainThread",
		                        "Failed to dump default config, "
		                        "exception follows.");
		multi_logger_->log_warn("FawkesMainThread", e);
	}

	if (load_plugins_)
		free(load_plugins_);
	if (default_plugin_)
		free(default_plugin_);

	delete time_wait_;
	delete loop_start_;
	delete loop_end_;

	delete mainloop_barrier_;
	delete mainloop_mutex_;
}

/** Start the thread and wait until once() completes.
 * This is useful to assure that all plugins are loaded before assuming that
 * startup is complete.
 */
void
FawkesMainThread::full_start()
{
	init_barrier_ = new Barrier(2);

	start(false);

	init_barrier_->wait();
	delete (init_barrier_);
	init_barrier_ = 0;
}

void
FawkesMainThread::once()
{
	// register to all syncpoints of the main loop
	std::vector<BlockedTimingAspect::WakeupHook> hooks;
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_THINK);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_SKILL);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_ACT);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC);
	hooks.push_back(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP);

	try {
		for (std::vector<BlockedTimingAspect::WakeupHook>::const_iterator it = hooks.begin();
		     it != hooks.end();
		     it++) {
			syncpoints_start_hook_.push_back(syncpoint_manager_->get_syncpoint(
			  "FawkesMainThread", BlockedTimingAspect::blocked_timing_hook_to_start_syncpoint(*it)));
			syncpoints_start_hook_.back()->register_emitter("FawkesMainThread");
			syncpoints_end_hook_.push_back(syncpoint_manager_->get_syncpoint(
			  "FawkesMainThread", BlockedTimingAspect::blocked_timing_hook_to_end_syncpoint(*it)));
		}
	} catch (Exception &e) {
		multi_logger_->log_error("FawkesMainThread", "Failed to acquire mainloop syncpoint");
		throw;
	}

	// if plugins passed on command line or in init options, load!
	if (load_plugins_) {
		try {
			plugin_manager_->load(load_plugins_);
		} catch (Exception &e) {
			multi_logger_->log_error("FawkesMainThread",
			                         "Failed to load plugins %s, "
			                         "exception follows",
			                         load_plugins_);
			multi_logger_->log_error("FawkesMainThread", e);
		}
	}

	// load extra default plugin given via init options
	try {
		if (default_plugin_ && (strcmp("default", default_plugin_) != 0)) {
			plugin_manager_->load(default_plugin_);
		}
	} catch (PluginLoadException &e) {
		if (e.plugin_name() != default_plugin_) {
			// only print if name is not default, i.e. one of the plugins that
			// the default meta plugin
			multi_logger_->log_error("FawkesMainThread",
			                         "Failed to load default "
			                         "plugins, exception follows");
			multi_logger_->log_error("FawkesMainThread", e);
		}
	}

	// if no specific plugins were given to load, load the default plugin
	if (!load_plugins_) {
		try {
			plugin_manager_->load("default");
		} catch (PluginLoadException &e) {
			if (e.plugin_name() != "default") {
				// only print if name is not default, i.e. one of the plugins that
				// the default meta plugin
				multi_logger_->log_error("FawkesMainThread",
				                         "Failed to load default "
				                         "plugins, exception follows");
				multi_logger_->log_error("FawkesMainThread", e);
			}
		} catch (Exception &e) {
			multi_logger_->log_error("FawkesMainThread",
			                         "Failed to load default "
			                         "plugins, exception follows");
			multi_logger_->log_error("FawkesMainThread", e);
		}
	}

	if (init_barrier_)
		init_barrier_->wait();
}

void
FawkesMainThread::set_mainloop_thread(Thread *mainloop_thread)
{
	loopinterrupt_antistarve_mutex->lock();
	mainloop_mutex_->lock();
	mainloop_barrier_->interrupt();
	mainloop_thread_ = mainloop_thread;
	mainloop_mutex_->unlock();
	loopinterrupt_antistarve_mutex->unlock();
}

void
FawkesMainThread::loop()
{
	if (!thread_manager_->timed_threads_exist()) {
		multi_logger_->log_debug("FawkesMainThread", "No timed threads exist, waiting");
		try {
			thread_manager_->wait_for_timed_threads();
			multi_logger_->log_debug("FawkesMainThread",
			                         "Timed threads have been added, "
			                         "running main loop now");
		} catch (InterruptedException &e) {
			multi_logger_->log_debug("FawkesMainThread", "Waiting for timed threads interrupted");
			return;
		}
	}

	plugin_manager_->lock();

	try {
		if (time_wait_) {
			time_wait_->mark_start();
		}
		loop_start_->stamp_systime();

		CancelState old_state;
		set_cancel_state(CANCEL_DISABLED, &old_state);

		mainloop_mutex_->lock();

		if (unlikely(mainloop_thread_ != NULL)) {
			try {
				if (likely(mainloop_thread_ != NULL)) {
					mainloop_thread_->wakeup(mainloop_barrier_);
					mainloop_barrier_->wait();
				}
			} catch (Exception &e) {
				multi_logger_->log_warn("FawkesMainThread", e);
			}
		} else {
			uint num_hooks = syncpoints_start_hook_.size();
			if (syncpoints_end_hook_.size() != num_hooks) {
				multi_logger_->log_error(
				  "FawkesMainThread",
				  "Hook syncpoints are not initialized properly, not waking up any threads!");
			} else {
				for (uint i = 0; i < num_hooks; i++) {
					syncpoints_start_hook_[i]->emit("FawkesMainThread");
					syncpoints_end_hook_[i]->reltime_wait_for_all("FawkesMainThread",
					                                              0,
					                                              max_thread_time_nanosec_);
				}
			}
		}
		mainloop_mutex_->unlock();
		set_cancel_state(old_state);

		test_cancel();

		thread_manager_->try_recover(recovered_threads_);
		if (!recovered_threads_.empty()) {
			// threads have been recovered!
			//multi_logger_->log_error(name(), "Threads recovered %zu", recovered_threads_.size());
			if (enable_looptime_warnings_) {
				if (recovered_threads_.size() == 1) {
					multi_logger_->log_warn("FawkesMainThread",
					                        "The thread %s could be "
					                        "recovered and resumes normal operation",
					                        recovered_threads_.front().c_str());
				} else {
					std::string s;
					for (std::list<std::string>::iterator i = recovered_threads_.begin();
					     i != recovered_threads_.end();
					     ++i) {
						s += *i + " ";
					}

					multi_logger_->log_warn("FawkesMainThread",
					                        "The following threads could be "
					                        "recovered and resumed normal operation: %s",
					                        s.c_str());
				}
			}
			recovered_threads_.clear();
		}

		if (desired_loop_time_sec_ > 0) {
			loop_end_->stamp_systime();
			float loop_time = *loop_end_ - loop_start_;
			if (enable_looptime_warnings_) {
				// give some extra 10% to eliminate frequent false warnings due to regular
				// time jitter (TimeWait might not be all that precise)
				if (loop_time > 1.1 * desired_loop_time_sec_) {
					multi_logger_->log_debug("FawkesMainThread",
					                        "Loop time exceeded, "
					                        "desired: %f sec (%u usec),  actual: %f sec",
					                        desired_loop_time_sec_,
					                        desired_loop_time_usec_,
					                        loop_time);
				}
			}
		}

		plugin_manager_->unlock();

		if (time_wait_) {
			time_wait_->wait_systime();
		} else {
			yield();
		}
	} catch (Exception &e) {
		multi_logger_->log_warn("FawkesMainThread",
		                        "Exception caught while executing default main "
		                        "loop, ignoring.");
		multi_logger_->log_warn("FawkesMainThread", e);
	} catch (std::exception &e) {
		multi_logger_->log_warn("FawkesMainThread",
		                        "STL Exception caught while executing default main "
		                        "loop, ignoring. (what: %s)",
		                        e.what());
	}
	// catch ... is not a good idea, would catch cancellation exception
	// at least needs to be rethrown.
}

/** Get logger.
 * @return logger
 */
MultiLogger *
FawkesMainThread::logger() const
{
	return multi_logger_;
}

/** @class FawkesMainThread::Runner <baseapp/main_thread.h>
 * Utility class to run the main thread.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param fmt Fawkes main thread to run
 * @param register_signals true to register default signal handlers
 * for SIGINT, SIGTERM, and SIGALRM.
 */
FawkesMainThread::Runner::Runner(FawkesMainThread *fmt, bool register_signals)
{
	init_mutex_       = new Mutex();
	init_running_     = true;
	init_quit_        = false;
	sigint_running_   = false;
	register_signals_ = register_signals;

	fmt_ = fmt;

	SignalManager::ignore(SIGPIPE);
	if (register_signals_) {
		SignalManager::register_handler(SIGINT, this);
		SignalManager::register_handler(SIGTERM, this);
		SignalManager::register_handler(SIGALRM, this);
	}
}

/** Destructor. */
FawkesMainThread::Runner::~Runner()
{
	if (register_signals_) {
		SignalManager::unregister_handler(SIGINT);
		SignalManager::unregister_handler(SIGTERM);
		SignalManager::unregister_handler(SIGALRM);
	}
	delete init_mutex_;
}

/** Run main thread. */
void
FawkesMainThread::Runner::run()
{
	init_mutex_->lock();
	init_running_ = false;
	if (!init_quit_) {
		fmt_->full_start();
		fmt_->logger()->log_info("FawkesMainThread",
		                         "Fawkes %s startup complete",
		                         FAWKES_VERSION_STRING);
		init_mutex_->unlock();
		fmt_->join();
	} else {
		init_mutex_->unlock();
	}
}

/** Handle signals.
 * @param signum signal number
 */
void
FawkesMainThread::Runner::handle_signal(int signum)
{
	if ((signum == SIGINT) && !sigint_running_) {
		MutexLocker lock(init_mutex_);
		if (init_running_) {
			init_quit_ = true;
		} else {
			fmt_->cancel();
		}
		sigint_running_ = true;
		alarm(3 /* sec */);
	} else if (signum == SIGALRM) {
		// we could use fmt_->logger()->log_info(), but we prefer direct printf
		// because we're mentioning Ctrl-C only useful on the console anyway
		printf("\nFawkes shutdown and finalization procedure still running.\n"
		       "Hit Ctrl-C again to force immediate exit.\n\n");

	} else if ((signum == SIGTERM) || sigint_running_) {
		// we really need to quit
		::exit(-2);
	}
}

} // end namespace fawkes
