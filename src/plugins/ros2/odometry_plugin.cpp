
/***************************************************************************
 *  odometry_plugin.cpp - Plugin to publish odometry to ROS
 *
 *  Created: Fri Jun 1 13:29:39 CEST
 *  Copyright  2012  Sebastian Reuter
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

#include "odometry_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to publish odometry to ROS. */
class ROS2OdometryPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2OdometryPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2OdometryThread());
	}
};

PLUGIN_DESCRIPTION("Send odometry data to ROS")
EXPORT_PLUGIN(ROS2OdometryPlugin)
