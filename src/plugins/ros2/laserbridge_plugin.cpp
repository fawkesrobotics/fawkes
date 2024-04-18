
/***************************************************************************
 *  laserbridge_plugin.cpp - Takes ros2 /scan topic and sends it to fawkes
 *  pcl_manager
 *
 *  Created: Tue May 29 19:30:47 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "laserbridge_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin exchange laser scans between Fawkes and ROS.
 * @author Tim Niemueller
 */
class Ros2LaserBridgePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit Ros2LaserBridgePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2LaserBridgeThread());
	}
};

PLUGIN_DESCRIPTION("ROS laser bridge exchange plugin")
EXPORT_PLUGIN(Ros2LaserBridgePlugin)
