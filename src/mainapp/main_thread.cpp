
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
#include <utils/system/argparser.h>
#include <utils/system/hostinfo.h>

#include <blackboard/blackboard.h>
#include <mainapp/thread_initializer.h>
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
  hostinfo = new HostInfo();

  /* Logging stuff */
  multi_logger = new MultiLogger( new ConsoleLogger() );
  LibLogger::init(multi_logger);

  /* Config stuff */
  config             = new SQLiteConfiguration(CONFDIR);

  if ( argp->hasArgument("c") ) {
    config_mutable_file = strdup(argp->getArgument("c"));
  } else {
    if ( asprintf(&config_mutable_file, "%s.db", hostinfo->short_name()) == -1 ) {
      config_mutable_file = strdup(hostinfo->short_name());
      printf("WARNING: could not asprintf local config file name, using short hostname\n");
    }
  }
  if ( argp->hasArgument("d") ) {
    config_default_file = argp->getArgument("d");
  } else {
    config_default_file = "default.db";
  }
  config->load(config_mutable_file, config_default_file);
  config_manager     = new FawkesConfigManager(config);
  blackboard         = new BlackBoard();
  thread_initializer = new FawkesThreadInitializer(blackboard, config, multi_logger);
  thread_manager     = new FawkesThreadManager(thread_initializer);
  plugin_manager     = new FawkesPluginManager(thread_manager);
  network_manager    = new FawkesNetworkManager(thread_manager, 1910);

  thread_initializer->set_fnet_hub( network_manager->hub() );

  plugin_manager->set_hub( network_manager->hub() );
  config_manager->set_hub( network_manager->hub() );
}


/** Destructor. */
FawkesMainThread::~FawkesMainThread()
{
  delete plugin_manager;
  delete blackboard;
  delete config_manager;
  delete config;
  free(config_mutable_file);
  delete hostinfo;
  delete network_manager;
  delete thread_manager;
  delete thread_initializer;

  // implicitly frees multi_logger and all sub-loggers
  LibLogger::finalize();
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
