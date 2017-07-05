
/***************************************************************************
 *  pddl_robot_memory_plugin.cpp - pddl_robot_memory
 *
 *  Plugin created: Thu Oct 13 13:34:05 2016

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

#include "pddl_robot_memory_thread.h"

using namespace fawkes;

/** @class PddlRobotMemoryPlugin "pddl_robot_memory_plugin.cpp"
 * Generate PDDL files from the robot memory
 * @author Frederik Zwilling
 */
class PddlRobotMemoryPlugin : public fawkes::Plugin
{
 public:
  /** Constructor
   * @param config Fakwes configuration
   */
  PddlRobotMemoryPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new PddlRobotMemoryThread());
  }
};

PLUGIN_DESCRIPTION("Generate PDDL files from the robot memory")
EXPORT_PLUGIN(PddlRobotMemoryPlugin)
