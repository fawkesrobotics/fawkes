
/***************************************************************************
 *  blackboard.cpp - BlackBoard plugin
 *
 *  Generated: Sat Sep 16 17:11:13 2006 (on train to Cologne)
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

#include <blackboard/blackboard.h>
#include <blackboard/main_thread.h>

/** Constructor. */
BlackBoardPlugin::BlackBoardPlugin()
{
  thread_list.push_back( new BlackBoardMainThread() );
}


/** Destructor. */
BlackBoardPlugin::~BlackBoardPlugin()
{
  for (ThreadList::iterator i = thread_list.begin(); i != thread_list.end(); ++i) {
    delete (*i);
  }
  thread_list.clear();
}


/** Get presistency.
 * @return true, BlackBoardPlugin is persistent and cannot be unloaded.
 */
bool
BlackBoardPlugin::persistent()
{
  return true;
}


/** Get list of threads.
 * @return list of threads for BlackBoard.
 */
ThreadList &
BlackBoardPlugin::threads()
{
  return thread_list;
}


/** Get plugin type.
 * @return Plugin::BLACKBOARD
 */
Plugin::PluginType
BlackBoardPlugin::type() const
{
  return Plugin::BLACKBOARD;
}


/** Get name of plugin.
 * @return BlackBoardPlugin
 */
const char *
BlackBoardPlugin::name() const
{
  return "BlackBoardPlugin";
}


/** Plugin factory function for this plugin.
 * @return an instance of TestPlugin
 */
extern "C"
Plugin *
plugin_factory()
{
  return new BlackBoardPlugin();
}


/** Plugin destruction function for this plugin.
 * @param plugin The plugin that is to be destroyed. Do not use this plugin
 *        afterwards
 */
extern "C"
void
plugin_destroy(Plugin *plugin)
{
  delete plugin;
}
