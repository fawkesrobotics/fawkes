
/***************************************************************************
 *  laser_plugin.cpp - Fawkes Laser Plugin
 *
 *  Created: Tue Aug 05 13:11:02 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <plugins/laser/laser_plugin.h>

#include "sensor_thread.h"
#include "lase_edl_aqt.h"
#include "urg_aqt.h"

using namespace fawkes;

/** @class LaserPlugin "laser_plugin.h"
 * Laser plugin for Fawkes.
 * This plugin integrates Fawkes with Laser, for example for accessing
 * a simulator.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LaserPlugin::LaserPlugin(Configuration *config)
  : Plugin(config)
{
  //LaseEdlAcquisitionThread *aqt = new LaseEdlAcquisitionThread();
  HokuyoUrgAcquisitionThread *aqt = new HokuyoUrgAcquisitionThread();
  thread_list.push_back(new LaserSensorThread(aqt));
  thread_list.push_back(aqt);
}


PLUGIN_DESCRIPTION("Reads data from laser range finder and writes to BlackBoard")
EXPORT_PLUGIN(LaserPlugin)
