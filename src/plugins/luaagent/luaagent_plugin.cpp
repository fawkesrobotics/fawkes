
/***************************************************************************
 *  luaagent_plugin.h - Fawkes LuaAgent Plugin
 *
 *  Created: Thu Jan 01 13:00:00 2008
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "luaagent_plugin.h"
#include "periodic_exec_thread.h"
#include "continuous_exec_thread.h"

using namespace fawkes;

/** @class LuaAgentPlugin <plugins/luaagent/luaagent_plugin.h>
 * LuaAgent Plugin.
 * This plugin runs an agent written in Lua.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LuaAgentPlugin::LuaAgentPlugin(Configuration *config)
  : Plugin(config)
{
  bool continuous = false;
  try {
    continuous = config->get_bool("/luaagent/continuous");
  } catch (Exception &e) {} // ignored, use default

  if (continuous) {
    thread_list.push_back(new LuaAgentContinuousExecutionThread());
  } else {
    thread_list.push_back(new LuaAgentPeriodicExecutionThread());
  }
}

PLUGIN_DESCRIPTION("Runs an agent written in Lua")
EXPORT_PLUGIN(LuaAgentPlugin)
