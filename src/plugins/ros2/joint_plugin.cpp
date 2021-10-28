
/***************************************************************************
 *  joint_plugin.cpp - Publish JointStates to ROS
 *
 *  Created: Wed Sep 25 18:27:26 2013
 *  Copyright  2013  Till Hofmann
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

#include "joint_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin publish JointStates to ROS
 * @author Till Hofmann
 */
class ROS2JointPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2JointPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2JointThread());
	}
};

PLUGIN_DESCRIPTION("ROS JointState Plugin")
EXPORT_PLUGIN(ROS2JointPlugin)
