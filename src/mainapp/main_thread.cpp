
/***************************************************************************
 *  main_thread.cpp - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:47:50 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <mainapp/main_thread.h>

#include <mainapp/network_manager.h>
#include <mainapp/thread_manager.h>

#include <core/exceptions/system.h>
#include <config/sqlite.h>
#include <config/net_handler.h>
#include <utils/logging/multi.h>
#include <utils/logging/console.h>
#include <utils/logging/liblogger.h>
#include <utils/logging/factory.h>
#include <utils/system/argparser.h>
#include <utils/time/clock.h>
#include <utils/time/wait.h>
#include <netcomm/utils/network_logger.h>

#include <blackboard/local.h>
#include <aspect/inifin.h>
#include <plugin/manager.h>
#include <plugin/net/handler.h>

#include <cstdio>
#include <cstring>

using namespace fawkes;

/** @class FawkesMainThread mainapp/main_thread.h
 * Fawkes main thread.
 * This thread initializes all important stuff like the BlackBoard,
 * handles plugins and wakes up threads at defined hooks.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param argp argument parser
 */
FawkesMainThread::FawkesMainThread(ArgumentParser *argp)
  : Thread("FawkesMainThread")
{
  __blackboard        = NULL;
  __config_nethandler = NULL;
  __config            = NULL;
  __plugin_manager      = NULL;
  __network_manager     = NULL;
  __thread_manager      = NULL;
  __aspect_inifin     = NULL;

  __mainloop          = this;

  __argp = argp;

  /* Logging stuff */
  const char *tmp;
  Logger::LogLevel log_level = Logger::LL_DEBUG;
  if ( __argp->has_arg("q") ) {
    log_level = Logger::LL_INFO;
    if ( (tmp = __argp->arg("q")) != NULL ) {
      for (unsigned int i = 0; i < strlen(tmp); ++i) {
	if ( tmp[i] == 'q' ) {
	  switch (log_level) {
	  case Logger::LL_INFO:  log_level = Logger::LL_WARN; break;
	  case Logger::LL_WARN:  log_level = Logger::LL_ERROR; break;
	  case Logger::LL_ERROR: log_level = Logger::LL_NONE; break;
	  default: break;
	  }
	}
      }
    }
  } else if ( (tmp = __argp->arg("l")) != NULL ) {
    if ( strcmp(tmp, "debug") == 0 ) {
      log_level = Logger::LL_DEBUG;
    } else if ( strcmp(tmp, "info") == 0 ) {
      log_level = Logger::LL_INFO;
    } else if ( strcmp(tmp, "warn") == 0 ) {
      log_level = Logger::LL_WARN;
    } else if ( strcmp(tmp, "error") == 0 ) {
      log_level = Logger::LL_ERROR;
    } else if ( strcmp(tmp, "none") == 0 ) {
      log_level = Logger::LL_NONE;
    } else {
      printf("Unknown log level '%s', using default\n", tmp);
    }
  }

  if ( (tmp = __argp->arg("L")) != NULL ) {
    try {
      __multi_logger = LoggerFactory::multilogger_instance(tmp);
    } catch (Exception &e) {
      e.append("Initializing multi logger failed");
      destruct();
      throw;
    }
  } else {
    __multi_logger = new MultiLogger(new ConsoleLogger());
  }

  __multi_logger->set_loglevel(log_level);
  LibLogger::init(__multi_logger);

  /* Config stuff */
  __config = new SQLiteConfiguration(CONFDIR);
  __config->load(__argp->arg("c"), __argp->arg("d"));

  /* Clock */
  __clock = Clock::instance();

  std::string bb_magic_token = "FawkesBlackBoard";
  unsigned int bb_size = 2097152;
  try {
    bb_magic_token = __config->get_string("/fawkes/mainapp/blackboard_magic_token");
  } catch (Exception &e) {
    __multi_logger->log_warn("FawkesMainApp", "BlackBoard magic token not defined. "
			     "Will use %s, saving to default DB", bb_magic_token.c_str());
    __config->set_default_string("/fawkes/mainapp/blackboard_magic_token",
				 bb_magic_token.c_str());
  }
  try {
    bb_size = __config->get_uint("/fawkes/mainapp/blackboard_size");
  } catch (Exception &e) {
    __multi_logger->log_warn("FawkesMainApp", "BlackBoard size not defined. "
			     "Will use %u, saving to default DB", bb_size);
    __config->set_default_uint("/fawkes/mainapp/blackboard_size", bb_size);
  }

  // Cleanup stale BlackBoard shared memory segments if requested
  if ( __argp->has_arg("C") ) {
    LocalBlackBoard::cleanup(bb_magic_token.c_str(), /* output with lister? */ true);
  }

  /* Managers */
  try {
    __blackboard         = new LocalBlackBoard(bb_size, bb_magic_token.c_str());
    __thread_manager     = new FawkesThreadManager();
    __aspect_inifin      = new AspectIniFin(__blackboard,
					    __thread_manager->aspect_collector(),
					    __config, __multi_logger, __clock);
    __thread_manager->set_inifin(__aspect_inifin, __aspect_inifin);
    __plugin_manager     = new PluginManager(__thread_manager, __config,
					     "/fawkes/meta_plugins/");
    __network_manager    = new FawkesNetworkManager(__thread_manager, 1910);
    __config_nethandler  = new ConfigNetworkHandler(__config, __network_manager->hub());
  } catch (Exception &e) {
    e.append("Initializing managers failed");
    destruct();
    throw;
  }

  __network_logger = new NetworkLogger(__network_manager->hub(), log_level);
  __multi_logger->add_logger(__network_logger);

  __aspect_inifin->set_fnet_hub( __network_manager->hub() );
  __aspect_inifin->set_network_members( __network_manager->nnresolver(),
					__network_manager->service_publisher(),
					__network_manager->service_browser() );
  __aspect_inifin->set_plugin_manager(__plugin_manager);
  __aspect_inifin->set_mainloop_employer(this);
  __aspect_inifin->set_logger_employer(this);
  __aspect_inifin->set_blocked_timing_executor(__thread_manager);

  __plugin_nethandler = new PluginNetworkHandler(__plugin_manager,
						 __network_manager->hub() );
  __plugin_nethandler->start();

  __blackboard->start_nethandler(__network_manager->hub());

  __loop_start = new Time(__clock);
  __loop_end   = new Time(__clock);
  try {
    __max_thread_time_usec = __config->get_uint("/fawkes/mainapp/max_thread_time");
  } catch (Exception &e) {
    __max_thread_time_usec = 30000;
    __multi_logger->log_info("FawkesMainApp", "Maximum thread time not set, assuming 30ms.");
  }

  __time_wait = NULL;
  try {
    __desired_loop_time_usec = __config->get_uint("/fawkes/mainapp/desired_loop_time");
    if ( __desired_loop_time_usec > 0 ) {
      __time_wait = new TimeWait(__clock, __desired_loop_time_usec);
    }
  } catch (Exception &e) {
    __desired_loop_time_usec = 0;
    __multi_logger->log_info("FawkesMainApp", "Minimum loop time not set, assuming 0");
  }
  __desired_loop_time_sec  = (float)__desired_loop_time_usec / 1000000.f;
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
  // Must delete network logger first since network manager has to die before the LibLogger
  // is finalized.
  __multi_logger->remove_logger(__network_logger);
  delete __network_logger;

  if ( __plugin_nethandler ) {
    __plugin_nethandler->cancel();
    __plugin_nethandler->join();
    delete __plugin_nethandler;
  }
  delete __plugin_manager;
  delete __blackboard;
  delete __config_nethandler;
  delete __config;
  delete __network_manager;
  delete __thread_manager;
  delete __aspect_inifin;
  delete __time_wait;
  delete __loop_start;
  delete __loop_end;

  // implicitly frees multi_logger and all sub-loggers
  LibLogger::finalize();

  Clock::finalize();
}

void
FawkesMainThread::once()
{
  if ( __argp->has_arg("p") ) {
    try {
      __plugin_manager->load(__argp->arg("p"));
    } catch (Exception &e) {
      __multi_logger->log_error("FawkesMainThread", "Failed to load plugins %s, "
				"exception follows", __argp->arg("p"));
      __multi_logger->log_error("FawkesMainThread", e);
    }
  } else {
    try {
      __plugin_manager->load("default");
    } catch (Exception &e) {
      // ignored, there is no default meta plugin set
    }
  }
}

void
FawkesMainThread::set_mainloop(MainLoop *mainloop)
{
  loopinterrupt_antistarve_mutex->lock();
  __thread_manager->interrupt_timed_thread_wait();
  loop_mutex->lock();
  if ( mainloop ) {
    __mainloop = mainloop;
  } else {
    __mainloop = this;
  }
  loop_mutex->unlock();
  loopinterrupt_antistarve_mutex->unlock();
}


void
FawkesMainThread::add_logger(Logger *logger)
{
  __multi_logger->add_logger(logger);
}


void
FawkesMainThread::remove_logger(Logger *logger)
{
  __multi_logger->remove_logger(logger);
}


void
FawkesMainThread::loop()
{
  __mainloop->mloop();
}


/** Thread loop.
 * Runs the main loop.
 */
void
FawkesMainThread::mloop()
{
  try {
    if ( ! __thread_manager->timed_threads_exist() ) {
      __multi_logger->log_debug("FawkesMainThread", "No timed threads exist, waiting");
      try {
	__thread_manager->wait_for_timed_threads();
	__multi_logger->log_debug("FawkesMainThread", "Timed threads have been added, "
				"running main loop now");
      } catch (InterruptedException &e) {
	return;
      }
    }

    if ( __time_wait ) {
      __time_wait->mark_start();
    }
    __loop_start->stamp_systime();

    try {
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP,       __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR,         __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS, __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE,     __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_THINK,          __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SKILL,          __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT,            __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC,       __max_thread_time_usec );
      __thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP,      __max_thread_time_usec );
    } catch (Exception &e) {
      __multi_logger->log_error("FawkesMainThread", e);
    }

    test_cancel();

    __thread_manager->try_recover(__recovered_threads);
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
      __multi_logger->log_warn("FawkesMainThread", "%s", s.c_str());
    }

    if (__desired_loop_time_sec > 0) {
      __loop_end->stamp_systime();
      float loop_time = *__loop_end - __loop_start;
      if (loop_time > __desired_loop_time_sec) {
	__multi_logger->log_warn("FawkesMainThread", "Loop time exceeded, "
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
