
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
#include <mainapp/thread_inifin.h>
#include <mainapp/plugin_manager.h>
#include <mainapp/network_manager.h>
#include <mainapp/thread_manager.h>

#ifdef USE_TIMETRACKER
#include <utils/time/tracker.h>
#endif

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
  plugin_manager      = NULL;
  __blackboard          = NULL;
  __config_nethandler   = NULL;
  __config              = NULL;
  network_manager     = NULL;
  thread_manager      = NULL;
  thread_inifin       = NULL;

  __argp = argp;

  /* Config stuff */
  __config             = new SQLiteConfiguration(CONFDIR);

  __config->load(__argp->arg("c"), __argp->arg("d"));

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

  /* Clock */
  __clock = Clock::instance();

  // Cleanup stale BlackBoard shared memory segments if requested
  if ( __argp->has_arg("C") ) {
    LocalBlackBoard::cleanup(__config->get_string("/fawkes/mainapp/blackboard_magic_token").c_str(),
			     /* output with lister? */ true);
  }

  /* Managers */
  try {
    __blackboard         = new LocalBlackBoard(__config->get_uint("/fawkes/mainapp/blackboard_size"),
					     __config->get_string("/fawkes/mainapp/blackboard_magic_token").c_str());
    thread_manager     = new FawkesThreadManager();
    thread_inifin      = new FawkesThreadIniFin(__blackboard,
						thread_manager->aspect_collector(),
						__config, __multi_logger, __clock);
    thread_manager->set_inifin(thread_inifin, thread_inifin);
    plugin_manager     = new FawkesPluginManager(thread_manager);
    network_manager    = new FawkesNetworkManager(thread_manager, 1910);
    __config_nethandler  = new ConfigNetworkHandler(__config, network_manager->hub());
  } catch (Exception &e) {
    e.append("Initializing managers failed");
    destruct();
    throw;
  }

  __network_logger = new NetworkLogger(network_manager->hub(), log_level);
  __multi_logger->add_logger(__network_logger);

  thread_inifin->set_fnet_hub( network_manager->hub() );
  thread_inifin->set_network_members( network_manager->nnresolver(),
				      network_manager->service_publisher(),
				      network_manager->service_browser() );

  plugin_manager->set_hub( network_manager->hub() );
  plugin_manager->start();

  __blackboard->start_nethandler(network_manager->hub());

  __time_wait = NULL;
  try {
    unsigned int min_loop_time = __config->get_uint("/fawkes/mainapp/min_loop_time");
    if ( min_loop_time > 0 ) {
      __time_wait = new TimeWait(__clock, min_loop_time);
    }
  } catch (Exception &e) {
    __multi_logger->log_info("FawkesMainApp", "Minimum loop time not set, assuming 0");
  }
#ifdef USE_TIMETRACKER
  __tt = NULL;
  try {
    if (__config->get_bool("/fawkes/mainapp/use_time_tracker") ) {
      __tt = new TimeTracker();
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
  } catch (Exception &e) {
    // ignored, if config value is missing we just don't start the time tracker
  }
#endif
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

  if ( plugin_manager ) {
    plugin_manager->cancel();
    plugin_manager->join();
    delete plugin_manager;
  }
  delete __blackboard;
  delete __config_nethandler;
  delete __config;
  delete network_manager;
  delete thread_manager;
  delete thread_inifin;
  delete __time_wait;
#ifdef USE_TIMETRACER
  delete __tt;
#endif

  // implicitly frees multi_logger and all sub-loggers
  LibLogger::finalize();

  Clock::finalize();
}

void
FawkesMainThread::once()
{
  if ( __argp->has_arg("p") ) {
    char *plugins = strdup(__argp->arg("p"));
    char *saveptr;
    char *plugin;

    plugin = strtok_r(plugins, ",", &saveptr);
    while ( plugin ) {
      try {
	plugin_manager->load(plugin);
      } catch (Exception &e) {
	__multi_logger->log_error("FawkesMainThread", "Failed to load plugin %s, "
				"exception follows", plugin);
	__multi_logger->log_error("FawkesMainThread", e);
      }
      plugin = strtok_r(NULL, ",", &saveptr);
    }

    free(plugins);
  }
}

#ifdef USE_TIMETRACKER
#define TIMETRACK_START(c1, c2, c3)		\
  if ( __tt ) {					\
    __tt->ping_start(c1);			\
    __tt->ping_start(c2);			\
    __tt->ping_start(c3);			\
  }
#define TIMETRACK_INTER(c1, c2)			\
  if ( __tt ) {					\
    __tt->ping_end(c1);			\
    __tt->ping_start(c2);			\
  }
#define TIMETRACK_END(c)			\
  if ( __tt ) {					\
    __tt->ping_end(c);				\
  }
#define TIMETRACK_OUTPUT			\
  if ( __tt && (++__tt_loopcount % 100) == 0) {	\
    __tt->print_to_stdout();\
  }
#else
#define TIMETRACK_START(c1, c2, c3)
#define TIMETRACK_INTER(c1, c2)
#define TIMETRACK_END(c)
#define TIMETRACK_OUTPUT
#endif

/** Thread loop.
 * Runs the main loop.
 */
void
FawkesMainThread::loop()
{
  if ( ! thread_manager->timed_threads_exist() ) {
    __multi_logger->log_debug("FawkesMainThread", "No threads exist, waiting");
    thread_manager->wait_for_timed_threads();
    __multi_logger->log_debug("FawkesMainThread", "Timed threads have been added, "
			                        "running main loop now");
  }

  TIMETRACK_START(__ttc_real_loop, __ttc_full_loop, __ttc_pre_loop);

  if ( __time_wait ) {
    __time_wait->mark_start();
  }

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP );

  TIMETRACK_INTER(__ttc_pre_loop, __ttc_sensor)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR );
  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS );

  TIMETRACK_INTER(__ttc_sensor, __ttc_worldstate)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE );

  TIMETRACK_INTER(__ttc_worldstate, __ttc_think)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_THINK );

  TIMETRACK_INTER(__ttc_think, __ttc_skill)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_SKILL );

  TIMETRACK_INTER(__ttc_skill, __ttc_act)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT );
  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC );

  TIMETRACK_INTER(__ttc_act, __ttc_post_loop)

  thread_manager->wakeup_and_wait( BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP );

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
  TIMETRACK_OUTPUT
}
