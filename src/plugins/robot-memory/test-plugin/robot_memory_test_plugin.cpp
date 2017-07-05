
/***************************************************************************
 *  robot_memory_test_plugin.cpp - robot_memory_test
 *
 *  Plugin created: Wed Aug 24 13:37:27 2016

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

#include "robot_memory_test_thread.h"

using namespace fawkes;

/** @class RobotMemoryTestPlugin "robot_memory_test_plugin.cpp"
 * gtests for the RobotMemory
 * @author Frederik Zwilling
 */
class RobotMemoryTestPlugin : public fawkes::Plugin
{
 public:
  /** Constructor
   * @param config Fakwes configuration
   */
  RobotMemoryTestPlugin(Configuration *config)
     : Plugin(config)
  {
     thread_list.push_back(new RobotMemoryTestThread());
  }
};

PLUGIN_DESCRIPTION("gtests for the RobotMemory")
EXPORT_PLUGIN(RobotMemoryTestPlugin)
