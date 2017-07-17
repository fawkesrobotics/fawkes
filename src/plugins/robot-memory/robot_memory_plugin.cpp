/***************************************************************************
 *  robot_memory_plugin.cpp - Fawkes Robot Memory Plugin
 *
 *  Created: Sun May 01 13:34:51 2016
 *  Copyright  2016  Frederik Zwilling
 *             2017  Tim Niemueller [www.niemueller.de]
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

#include "robot_memory_thread.h"
#include <string>
#include <logging/console.h>

#include <core/plugin.h>

using namespace fawkes;

/** Robot Memory Plugin.
 * This plugin provides a robot memory with mongodb
 *
 * @author Frederik Zwilling
 */
class RobotMemoryPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotMemoryPlugin(Configuration *config) : Plugin(config)
  {
	  thread_list.push_back(new RobotMemoryThread());
  }
};


PLUGIN_DESCRIPTION("Robot Memory based on MongoDB")
EXPORT_PLUGIN(RobotMemoryPlugin)
