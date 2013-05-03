/***************************************************************************
 *  robotino-sim_plugin.cpp - Plugin simulates the Robotino in Gazebo
 *
 *  Created: Fr 3. Mai 21:13:45 CEST 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "robotino_sim_thread.h"

using namespace fawkes;

/** Plugin to simulate the Robotino in Gazebo
 *
 *
 * @author Frederik Zwilling
 */
class RobotinoSimPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoSimPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RobotinoSimThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of Robotino in Gazebo")
EXPORT_PLUGIN(RobotinoSimPlugin)
