
/***************************************************************************
 *  motorinterface_plugin.cpp - Translates transrot motor messages to ROS2 Twist
 *
 *  Created: Sun Mar 31 13:29:39 CEST 2024
 *  Copyright  2024 Tim Wendt
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

#include "motorinterface_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to translate ROS2 Twist messages to Navigator transrot. */
class ROS2MotorInterfacePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2MotorInterfacePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2MotorInterfaceThread());
	}
};

PLUGIN_DESCRIPTION("Translate ROS2 Twist to Fawkes TransRot")
EXPORT_PLUGIN(ROS2MotorInterfacePlugin)
