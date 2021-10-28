
/***************************************************************************
 *  ros_plugin.cpp - Plugin to access ROS features
 *
 *  Created: Thu May 05 18:33:32 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "ros2_plugin.h"

#include "node_thread.h"

using namespace fawkes;

/** @class ROS2Plugin "ros2_plugin.h"
 * Plugin to access ROS from Fawkes.
 * This plugin integrates ROS and provides access to the ROS context
 * to other plugins.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
ROS2Plugin::ROS2Plugin(Configuration *config) : Plugin(config)
{
	thread_list.push_back(new ROS2NodeThread());
}

PLUGIN_DESCRIPTION("Provides access to ROS2")
EXPORT_PLUGIN(ROS2Plugin)
