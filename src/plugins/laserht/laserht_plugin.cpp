
/***************************************************************************
 *  laser_plugin.cpp - Fawkes Laser Plugin
 *
 *  Created: Sat Jul 04 21:34:23 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: laser_plugin.cpp 2546 2009-06-15 16:59:54Z tim $
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

#include "laser_plugin.h"

#include "sensor_thread.h"

using namespace fawkes;

/** @class LaserLinePlugin "laser_plugin.h"
 * Laser plugin for Fawkes.
 * This plugin integrates Fawkes with Laser, for example for accessing
 * a simulator.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LaserLinePlugin::LaserLinePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new LaserLineSensorThread());
}


PLUGIN_DESCRIPTION("Tries to fit a line through laser readings")
EXPORT_PLUGIN(LaserLinePlugin)
