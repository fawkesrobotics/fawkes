
/***************************************************************************
 *  worldmodel_plugin.cpp - Fawkes TimeTracker MainLoop Plugin
 *
 *  Created: Sat Aug 02 13:53:54 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/ttmainloop/ttmainloop_plugin.h>
#include <plugins/ttmainloop/thread.h>

using namespace fawkes;

/** @class TimeTrackerMainLoopPlugin ttmainloop_plugin.h
 * Simple TimeTracker MainLoop plugin.
 * This plugin facilitates the Fawkes TimeTracker main loop plugin. It provides
 * a main loop similar to the default main loop but augmented with a TimeTracker,
 * that measures the runtime of the different blocked timing hook execution.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
TimeTrackerMainLoopPlugin::TimeTrackerMainLoopPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new TimeTrackerMainLoopThread());
}

PLUGIN_DESCRIPTION("Replaces main loop with time tracked loop")
EXPORT_PLUGIN(TimeTrackerMainLoopPlugin)
