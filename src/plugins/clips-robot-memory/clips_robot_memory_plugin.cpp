
/***************************************************************************
 *  clips_robot_memory_plugin.cpp - clips_robot_memory
 *
 *  Plugin created: Mon Aug 29 15:41:47 2016

 *  Copyright  2016  Frederik Zwilling
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

#include <core/plugin.h>

#include "clips_robot_memory_thread.h"

using namespace fawkes;

/** @class ClipsRobotMemoryPlugin "clips_robot_memory_plugin.cpp"
 * CLIPS feature to access the robot memory
 * @author Frederik Zwilling
 */
class ClipsRobotMemoryPlugin : public fawkes::Plugin
{
 public:
  /** Constructor
   * @param config Fakwes configuration
   */
  ClipsRobotMemoryPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new ClipsRobotMemoryThread());
  }
};

PLUGIN_DESCRIPTION("CLIPS feature to access the robot memory")
EXPORT_PLUGIN(ClipsRobotMemoryPlugin)
