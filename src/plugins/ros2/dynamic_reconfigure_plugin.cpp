
/***************************************************************************
 *  dynamic_reconfigure_plugin.cpp - Robotino ROS Dynamic Reconfigure Plugin
 *
 *  Created: Fri May 05 20:07:27 2017
 *  Copyright  2017  Christoph Henke
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

#include "dynamic_reconfigure_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Send dynamic reconfigure commands to ROS.
 * @author Christoph Henke
 */
class ROS2DynamicReconfigurePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2DynamicReconfigurePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2DynamicReconfigureThread());
	}
};

PLUGIN_DESCRIPTION("Send dynamic reconfigure commands to ROS")
EXPORT_PLUGIN(ROS2DynamicReconfigurePlugin)
