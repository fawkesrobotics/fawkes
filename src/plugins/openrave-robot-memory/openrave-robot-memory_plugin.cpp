
/***************************************************************************
 *  openrave-robot-memory_plugin.cpp - openrave-robot-memory
 *
 *  Created: Thu Nov 24 13:14:33 2016
 *  Copyright  2016  Frederik Zwilling
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

#include <core/plugin.h>

#include "openrave-robot-memory_thread.h"

using namespace fawkes;

/** @class OpenraveRobotMemoryPlugin "openrave-robot-memory_plugin.cpp"
 * Creates an OpenRave Scene for motion planning from data in the robot memory
 * @author Frederik Zwilling
 */
class OpenraveRobotMemoryPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fakwes configuration
   */
  OpenraveRobotMemoryPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new OpenraveRobotMemoryThread());
  }
};

PLUGIN_DESCRIPTION("Creates an OpenRave Scene for motion planning from data in the robot memory")
EXPORT_PLUGIN(OpenraveRobotMemoryPlugin)
