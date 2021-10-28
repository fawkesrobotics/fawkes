
/***************************************************************************
 *  move_base_plugin.cpp - Emulate ROS move_base
 *
 *  Created: Wed May 07 13:47:29 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "move_base_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Send locomotion commands to ROS.
 * @author Sebastian Reuter
 */
class ROS2MoveBasePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2MoveBasePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2MoveBaseThread());
	}
};

PLUGIN_DESCRIPTION("Accept move_base commands from ROS")
EXPORT_PLUGIN(ROS2MoveBasePlugin)
