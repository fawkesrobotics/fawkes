
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
#include <mainapp/thread_manager.h>
#include <core/plugin.h>
#include <utils/plugin/plugin_loader.h>
#include <blackboard/interface_manager.h>

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
  plugin_loader = new PluginLoader(PLUGINDIR);
  Plugin *bb_plugin = plugin_loader->load("blackboard");
  plugins.clear();
  plugins["blackboard"] = bb_plugin;

  ThreadList &tl = bb_plugin->threads();

  thread_manager = new FawkesThreadManager();

  thread_manager->add(tl);

  interface_manager = thread_manager->getInterfaceManager();
}


/** Destructor. */
FawkesMainThread::~FawkesMainThread()
{
  delete thread_manager;

  // Unload all plugins
  for (std::map< std::string, Plugin * >::iterator pit = plugins.begin(); pit != plugins.end(); ++pit) {
    plugin_loader->unload((*pit).second);
  }

  delete plugin_loader;
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

  usleep(0);
}
