
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

#include <baseapp/main_thread.h>

#include <core/threading/interruptible_barrier.h>
#include <core/threading/mutex_locker.h>
#include <core/exceptions/system.h>
#include <core/version.h>
#include <config/config.h>
#include <utils/time/clock.h>
#include <utils/time/wait.h>
#include <netcomm/fawkes/network_manager.h>
#include <blackboard/local.h>

#include <aspect/manager.h>
#include <plugin/manager.h>
#include <plugin/loader.h>
#include <plugin/net/handler.h>

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <core/macros.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
 * @param plugin_manager plugin manager to load the desired plugins
 * @param load_plugins string with comma-separated list of names of plugins
 * to load on startup.
 * @param default_plugin additional default plugin name
 */
FawkesMainThread::FawkesMainThread(Configuration *config,
				   MultiLogger *multi_logger,
				   ThreadManager *thread_manager,
				   PluginManager *plugin_manager,
				   const char *load_plugins,
                                   const char *default_plugin)
  : Thread("FawkesMainThread")
{
  __plugin_manager    = plugin_manager;
  __thread_manager    = thread_manager;
  __multi_logger      = multi_logger;
  __config            = config;

  __mainloop_thread   = NULL;
  __mainloop_mutex    = new Mutex();
  __mainloop_barrier  = new InterruptibleBarrier(__mainloop_mutex, 2);

  __load_plugins      = NULL;
  if (load_plugins) {
    __load_plugins = strdup(load_plugins);
  }

  __default_plugin    = NULL;
  if (default_plugin) {
    __default_plugin = strdup(default_plugin);
  }

  /* Clock */
  __clock = Clock::instance();

  __loop_start = new Time(__clock);
  __loop_end   = new Time(__clock);
  try {
    __max_thread_time_usec = __config->get_uint("/fawkes/mainapp/max_thread_time");
  } catch (Exception &e) {
    __max_thread_time_usec = 30000;
    __multi_logger->log_info("FawkesMainApp",
			     "Maximum thread time not set, assuming 30ms.");
  }
  __max_thread_time_nanosec = __max_thread_time_usec * 1000;

  __time_wait = NULL;
  try {
    __desired_loop_time_usec =
      __config->get_uint("/fawkes/mainapp/desired_loop_time");
    if ( __desired_loop_time_usec > 0 ) {
      __time_wait = new TimeWait(__clock, __desired_loop_time_usec);
    }
  } catch (Exception &e) {
    __desired_loop_time_usec = 0;
    __multi_logger->log_info("FawkesMainApp",
			     "Desired loop time not set, assuming 0");
  }

  __desired_loop_time_sec  = (float)__desired_loop_time_usec / 1000000.f;

  try {
    __enable_looptime_warnings =
      __config->get_bool("/fawkes/mainapp/enable_looptime_warnings");
    if(!__enable_looptime_warnings) {
      __multi_logger->log_debug(name(), "loop time warnings are disabled");
    }
  } catch(Exception &e) {
    __enable_looptime_warnings = true;
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
    __config->try_dump();
  } catch (CouldNotOpenFileException &e) {
    if (e.get_errno() == EACCES) {
      __multi_logger->log_warn("FawkesMainThread", "Cannot write to dump file, "
			       "no write ");
      __multi_logger->log_warn("FawkesMainThread", "permission for file or "
			       "directory. This");
      __multi_logger->log_warn("FawkesMainThread", "usually happens if running "
			       "with system-wide");
      __multi_logger->log_warn("FawkesMainThread", "installed Fawkes as non-root "
			       "user. Make");
      __multi_logger->log_warn("FawkesMainThread", "configuration changes to the "
			       "host-based");
      __multi_logger->log_warn("FawkesMainThread", "database (set as non-default "
			       "values).");
    } else {
      __multi_logger->log_warn("FawkesMainThread", "Failed to dump default "
			       "config (open), exception follows.");
      __multi_logger->log_warn("FawkesMainThread", e);
    }
  } catch (Exception &e) {
    __multi_logger->log_warn("FawkesMainThread", "Failed to dump default config, "
			     "exception follows.");
    __multi_logger->log_warn("FawkesMainThread", e);
  }

  if (__load_plugins)   free(__load_plugins);
  if (__default_plugin) free(__default_plugin);

  delete __time_wait;
  delete __loop_start;
  delete __loop_end;

  delete __mainloop_barrier;
  delete __mainloop_mutex;
}

/** Start the thread and wait until once() completes.
 * This is useful to assure that all plugins are loaded before assuming that
 * startup is complete.
 */
void
FawkesMainThread::full_start()
{
  __init_barrier = new Barrier(2);
  
  start(false);

  __init_barrier->wait();
  delete(__init_barrier);
  __init_barrier = 0;
}

void
FawkesMainThread::once()
{
  // if plugins passed on command line or in init options, load!
  if ( __load_plugins) {
    try {
      __plugin_manager->load(__load_plugins);
    } catch (Exception &e) {
      __multi_logger->log_error("FawkesMainThread", "Failed to load plugins %s, "
				"exception follows", __load_plugins);
      __multi_logger->log_error("FawkesMainThread", e);
    }
  }

  // load extra default plugin given via init options
  try {
    if (__default_plugin && (strcmp("default", __default_plugin) != 0)) {
      __plugin_manager->load(__default_plugin);
    }
  } catch (PluginLoadException &e) {
    if (e.plugin_name() != __default_plugin) {
      // only print if name is not default, i.e. one of the plugins that
      // the default meta plugin
      __multi_logger->log_error("FawkesMainThread", "Failed to load default "
                                "plugins, exception follows");
      __multi_logger->log_error("FawkesMainThread", e);
    }
  }

  // if no specific plugins were given to load, load the default plugin
  if (! __load_plugins) {
    try {
      __plugin_manager->load("default");
    } catch (PluginLoadException &e) {
      if (e.plugin_name() != "default") {
	// only print if name is not default, i.e. one of the plugins that
	// the default meta plugin
	__multi_logger->log_error("FawkesMainThread", "Failed to load default "
				  "plugins, exception follows");
	__multi_logger->log_error("FawkesMainThread", e);
      }
    } catch (Exception &e) {
      __multi_logger->log_error("FawkesMainThread", "Failed to load default "
				"plugins, exception follows");
      __multi_logger->log_error("FawkesMainThread", e);
    }
  }

  if (__init_barrier)  __init_barrier->wait();
}

void
FawkesMainThread::set_mainloop_thread(Thread *mainloop_thread)
{
  loopinterrupt_antistarve_mutex->lock();
  __mainloop_mutex->lock();
  __mainloop_barrier->interrupt();
  __mainloop_thread = mainloop_thread;
  __mainloop_mutex->unlock();
  loopinterrupt_antistarve_mutex->unlock();
}


void
FawkesMainThread::loop()
{
  if ( ! __thread_manager->timed_threads_exist() ) {
    __multi_logger->log_debug("FawkesMainThread", "No timed threads exist, waiting");
    try {
      __thread_manager->wait_for_timed_threads();
      __multi_logger->log_debug("FawkesMainThread", "Timed threads have been added, "
				"running main loop now");
    } catch (InterruptedException &e) {
      __multi_logger->log_debug("FawkesMainThread", "Waiting for timed threads interrupted");
      return;
    }
  }

  __plugin_manager->lock();

  try {
    if ( __time_wait ) {
      __time_wait->mark_start();
    }
    __loop_start->stamp_systime();
      
    CancelState old_state;
    set_cancel_state(CANCEL_DISABLED, &old_state);

    __mainloop_mutex->lock();

    if (unlikely(__mainloop_thread != NULL)) {
      try {
	if (likely(__mainloop_thread != NULL)) {
	  __mainloop_thread->wakeup(__mainloop_barrier);
	  __mainloop_barrier->wait();
	}
      } catch (Exception &e) {
	__multi_logger->log_warn("FawkesMainThread", e);
      }
    } else {
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP,       __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE, __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE, __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS, __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE,     __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_THINK,          __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_SKILL,          __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_ACT,            __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC,       __max_thread_time_usec);
      safe_wake(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP,      __max_thread_time_usec);
    }
    __mainloop_mutex->unlock();
    set_cancel_state(old_state);

    test_cancel();

    __thread_manager->try_recover(__recovered_threads);
    if ( ! __recovered_threads.empty() ) {
      // threads have been recovered!
      //__multi_logger->log_error(name(), "Threads recovered %zu", __recovered_threads.size());
      if(__enable_looptime_warnings) {
	if ( __recovered_threads.size() == 1 ) {
	  __multi_logger->log_warn("FawkesMainThread", "The thread %s could be "
				   "recovered and resumes normal operation",
				   __recovered_threads.front().c_str());
	} else {
	  std::string s;
	  for (std::list<std::string>::iterator i = __recovered_threads.begin();
	       i != __recovered_threads.end(); ++i) {
	    s += *i + " ";
	  }
          
	  __multi_logger->log_warn("FawkesMainThread", "The following threads could be "
				   "recovered and resumed normal operation: %s", s.c_str());
	}
      }
      __recovered_threads.clear();
    }

    if (__desired_loop_time_sec > 0) {
      __loop_end->stamp_systime();
      float loop_time = *__loop_end - __loop_start;
      if(__enable_looptime_warnings) {
        // give some extra 10% to eliminate frequent false warnings due to regular
        // time jitter (TimeWait might not be all that precise)
	if (loop_time > 1.1 * __desired_loop_time_sec) {
	  __multi_logger->log_warn("FawkesMainThread", "Loop time exceeded, "
				   "desired: %f sec (%u usec),  actual: %f sec",
				   __desired_loop_time_sec, __desired_loop_time_usec,
				   loop_time);
	}
      }
    }

    __plugin_manager->unlock();

    if ( __time_wait ) {
      __time_wait->wait_systime();
    } else {
      yield();
    }
  } catch (Exception &e) {
    __multi_logger->log_warn("FawkesMainThread",
			     "Exception caught while executing default main "
			     "loop, ignoring.");
    __multi_logger->log_warn("FawkesMainThread", e);
  } catch (std::exception &e) {
    __multi_logger->log_warn("FawkesMainThread",
			     "STL Exception caught while executing default main "
			     "loop, ignoring. (what: %s)", e.what());
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
  return __multi_logger;
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
  __init_mutex       = new Mutex();
  __init_running     = true;
  __init_quit        = false;
  __sigint_running   = false;
  __register_signals = register_signals;

  __fmt = fmt;

  if (__register_signals) {
    SignalManager::register_handler(SIGINT,  this);
    SignalManager::register_handler(SIGTERM, this);
    SignalManager::register_handler(SIGALRM, this);
  }
}


/** Destructor. */
FawkesMainThread::Runner::~Runner()
{
  if (__register_signals) {
    SignalManager::unregister_handler(SIGINT);
    SignalManager::unregister_handler(SIGTERM);
    SignalManager::unregister_handler(SIGALRM);
  }
  delete __init_mutex;
}

/** Run main thread. */
void
FawkesMainThread::Runner::run()
{
  __init_mutex->lock();
  __init_running = false;
  if ( ! __init_quit ) {
    __fmt->full_start();
    __fmt->logger()->log_info("FawkesMainThread", "Fawkes %s startup complete",
                              FAWKES_VERSION_STRING);
    __init_mutex->unlock();
    __fmt->join();
  } else {
    __init_mutex->unlock();
  }
}

/** Handle signals.
 * @param signum signal number
 */
void
FawkesMainThread::Runner::handle_signal(int signum)
{
  if ((signum == SIGINT) && ! __sigint_running) {
    MutexLocker lock(__init_mutex);
    if (__init_running) {
      __init_quit = true;
    } else {
      __fmt->cancel();
    }
    __sigint_running = true;
    alarm(3 /* sec */);
  } else if (signum == SIGALRM) {
    // we could use __fmt->logger()->log_info(), but we prefer direct printf
    // because we're mentioning Ctrl-C only useful on the console anyway
    printf("\nFawkes shutdown and finalization procedure still running.\n"
           "Hit Ctrl-C again to force immediate exit.\n\n");

  } else if ((signum == SIGTERM) || __sigint_running) {
    // we really need to quit
    ::exit(-2);
  }
}

} // end namespace fawkes
