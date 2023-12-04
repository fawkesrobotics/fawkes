/***************************************************************************
 *  navgraph_plugin.cpp - Relay messages between Navgraph Blackboard Interfaces
 *                        and ROS2
 *
 *  Created: Dec 2023
 *  Copyright  2023  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#include "navgraph_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to relay messages between Navgraph Blackboard Interfaces and ROS2
 */
class ROS2NavgraphPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
    * @param config Fawkes configuration
    */
	explicit ROS2NavgraphPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2NavgraphThread());
	}
};

PLUGIN_DESCRIPTION("ROS CX Navgraph Blackboard Plugin")
EXPORT_PLUGIN(ROS2NavgraphPlugin)
