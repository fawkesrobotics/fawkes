/***************************************************************************
 *  robot_description_plugin.cpp - ROS Robot Description Plugin
 *
 *  Created: Fri May 16 15:26:33 2014
 *  Copyright  2014  Till Hofmann
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

#include "robot_description_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to publish the robot description to ROS
 * @author Till Hofmann
 */
class ROSRobotDescriptionPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROSRobotDescriptionPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2RobotDescriptionThread());
	}
};

PLUGIN_DESCRIPTION("Plugin to publish the robot description to ROS")
EXPORT_PLUGIN(ROSRobotDescriptionPlugin)
