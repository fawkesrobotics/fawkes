
/***************************************************************************
 *  example_plugin.cpp - Fawkes Example Plugin
 *
 *  Generated: Wed Nov 22 17:06:33 2006
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/example/example_plugin.h>
#include <plugins/example/thread.h>

#include <stdio.h>

/** @class ExamplePlugin plugins/example/example_plugin.h
 * Simple example plugin.
 * Creates a few threads for different wakeup hooks that print out messages
 * every 10 iterations.
 *
 * @author Tim Niemueller
 */

/** Modulo count */
#define MODC 100

/** Constructor. */
ExamplePlugin::ExamplePlugin()
{
  printf("ExamplePlugin constructor called\n");
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP,
					  "PreLoopThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_SENSOR,
					  "SensorThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE,
					  "WorldStateThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_THINK,
					  "ThinkThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_SKILL,
					  "SkillThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_ACT,
					  "ActThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP,
					  "PostLoopThread", MODC));
}

/** Destructor, prints out info message */
ExamplePlugin::~ExamplePlugin()
{
  printf("ExamplePlugin destructor called\n");
  for (ThreadList::iterator i = thread_list.begin(); i != thread_list.end(); ++i) {
    delete *i;
  }
}

/** Get the type of the plugin.
 * @return type of plugin
 */
Plugin::PluginType
ExamplePlugin::type() const
{
  return Plugin::MOTION;
}

/** Get the name of the plugin.
 * @return name of the plugin
 */
const char *
ExamplePlugin::name() const
{
  return "ExamplePlugin";
}

ThreadList &
ExamplePlugin::threads()
{
  return thread_list;
}




/** Plugin factory function for this plugin.
 * @return an instance of ExamplePlugin
 */
extern "C"
Plugin *
plugin_factory()
{
  return new ExamplePlugin();
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
