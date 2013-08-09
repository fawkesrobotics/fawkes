/***************************************************************************
 *  gazsim_laser_plugin.cpp - Plugin simulates a Hokuyo Laser Range finder in Gazebo
 *
 *  Created: Thu Aug 08 15:44:32 2013
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

#include "gazsim_laser_thread.h"

using namespace fawkes;

/** Plugin to simulate the Hokuyo in Gazebo
 *
 *
 * @author Frederik Zwilling
 */
class GazsimLaserPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimLaserPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new LaserSimThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of the Hokuyo in Gazebo")
EXPORT_PLUGIN(GazsimLaserPlugin)
