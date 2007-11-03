
/***************************************************************************
 *  main_thread.cpp - Fawkes main thread
 *
 *  Generated: Thu Nov  2 16:47:50 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <mainapp/main_thread.h>

#include <config/sqlite.h>
#include <utils/logging/multi.h>
#include <utils/logging/console.h>
#include <utils/logging/liblogger.h>
#include <utils/logging/factory.h>
#include <utils/system/argparser.h>
#include <utils/system/hostinfo.h>
#include <utils/time/clock.h>

#include <blackboard/blackboard.h>
#include <mainapp/thread_inifin.h>
#include <mainapp/plugin_manager.h>
#include <mainapp/network_manager.h>
#include <mainapp/config_manager.h>
#include <mainapp/thread_manager.h>

#include <stdio.h>

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
  blackboard          = NULL;
  config_manager      = NULL;
  config              = NULL;
  config_mutable_file = NULL;
  hostinfo            = NULL;
  network_manager     = NULL;
  thread_manager      = NULL;
  thread_inifin       = NULL;

  hostinfo = new HostInfo();

  /* Config stuff */
  config             = new SQLiteConfiguration(CONFDIR);

  if ( argp->has_arg("c") ) {
    config_mutable_file = strdup(argp->arg("c"));
  } else {
    if ( asprintf(&config_mutable_file, "%s.db", hostinfo->short_name()) == -1 ) {
      config_mutable_file = strdup(hostinfo->short_name());
      printf("WARNING: could not asprintf local config file name, using short hostname\n");
    }
  }
  if ( argp->has_arg("d") ) {
    config_default_file = argp->arg("d");
  } else {
    config_default_file = "default.db";
  }
  config->load(config_mutable_file, config_default_file);

  /* Logging stuff */
  const char *tmp;
  Logger::LogLevel log_level = Logger::LL_DEBUG;
  if ( argp->has_arg("q") ) {
    log_level = Logger::LL_INFO;
    if ( (tmp = argp->arg("q")) != NULL ) {
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
  } else if ( (tmp = argp->arg("l")) != NULL ) {
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

  if ( (tmp = argp->arg("L")) != NULL ) {
    try {
      multi_logger = LoggerFactory::multilogger_instance(tmp);
    } catch (Exception &e) {
      e.append("Initializing multi logger failed");
      destruct();
      throw;
    }
  } else {
    multi_logger = new MultiLogger(new ConsoleLogger());
  }

  multi_logger->set_loglevel(log_level);
  LibLogger::init(multi_logger);

  /* Clock */
  clock = Clock::instance();

  /* Managers */
  try {
    config_manager     = new FawkesConfigManager(config);
    blackboard         = new BlackBoard();
    thread_inifin      = new FawkesThreadIniFin(blackboard, config, multi_logger, clock);
    thread_manager     = new FawkesThreadManager(thread_inifin, thread_inifin);
    plugin_manager     = new FawkesPluginManager(thread_manager);
    network_manager    = new FawkesNetworkManager(thread_manager, 1910);
  } catch (Exception &e) {
    e.append("Initializing managers failed");
    destruct();
    throw;
  }

  thread_inifin->set_fnet_hub( network_manager->hub() );
  thread_inifin->set_network_members( network_manager->nnresolver(),
				      network_manager->service_publisher(),
				      network_manager->service_browser() );

  plugin_manager->set_hub( network_manager->hub() );
  config_manager->set_hub( network_manager->hub() );
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
  delete plugin_manager;
  delete blackboard;
  delete config_manager;
  delete config;
  if ( config_mutable_file != NULL )  free(config_mutable_file);
  delete hostinfo;
  delete network_manager;
  delete thread_manager;
  delete thread_inifin;

  // implicitly frees multi_logger and all sub-loggers
  LibLogger::finalize();

  Clock::finalize();
}


/** Thread loop.
 * Runs the main loop.
 */
void
FawkesMainThread::loop()
{
  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_SENSOR );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_SENSOR );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_THINK );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_THINK );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_SKILL );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_SKILL );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_ACT );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_ACT );

  thread_manager->wakeup( BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP );
  thread_manager->wait(   BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP );

  network_manager->process();

  test_cancel();
  usleep(0);
}
