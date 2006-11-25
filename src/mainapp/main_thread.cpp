
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

#include <core/threading/thread_manager.h>
#include <blackboard/blackboard.h>
#include <mainapp/thread_initializer.h>
#include <mainapp/plugin_manager.h>
#include <mainapp/network_manager.h>

#include <stdio.h>

/** @class FawkesMainThread mainapp/main_thread.h
 * Fawkes main thread.
 * This thread initializes all important stuff like the BlackBoard,
 * handles plugins and wakes up threads at defined hooks.
 *
 * @author Tim Niemueller
 */


/** Constructor. */
FawkesMainThread::FawkesMainThread()
{
  blackboard         = new BlackBoard();
  thread_initializer = new FawkesThreadInitializer(blackboard);
  thread_manager     = new ThreadManager(thread_initializer);
  plugin_manager     = new FawkesPluginManager(thread_manager);
  network_manager    = new FawkesNetworkManager(thread_manager, 1910);

  network_manager->add_handler(plugin_manager);
}


/** Destructor. */
FawkesMainThread::~FawkesMainThread()
{
  delete network_manager;
  delete plugin_manager;
  delete thread_manager;
  delete thread_initializer;
  delete blackboard;
}


/** Thread loop.
 * Runs the main loop.
 */
void
FawkesMainThread::loop()
{
  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_PRE_LOOP );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_PRE_LOOP );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_SENSOR );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_SENSOR );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_WORLDSTATE );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_WORLDSTATE );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_THINK );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_THINK );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_SKILL );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_SKILL );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_ACT );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_ACT );

  thread_manager->wakeup( FawkesThread::WAKEUP_HOOK_POST_LOOP );
  thread_manager->wait(   FawkesThread::WAKEUP_HOOK_POST_LOOP );

  // Load plugins that have been requested in this loop
  plugin_manager->load();
}
