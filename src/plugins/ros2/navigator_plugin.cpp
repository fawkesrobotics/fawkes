
/***************************************************************************
 *  navigator_plugin.cpp - Robotino ROS2 Navigator Plugin
 *
 *  Created: Sat June 09 15:13:27 2012
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

#include "navigator_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Send locomotion commands to ROS2.
 * @author Sebastian Reuter
 */
class ROS2NavigatorPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2NavigatorPlugin(Configuration *config) : Plugin(config)
	{
		std::string prefix     = "/ros2/navigator";
		std::string cfg_prefix = prefix + "/";

		thread_list.push_back(new ROS2NavigatorThread(cfg_prefix));
	}
};

PLUGIN_DESCRIPTION("Send locomotion commands to ROS2")
EXPORT_PLUGIN(ROS2NavigatorPlugin)
