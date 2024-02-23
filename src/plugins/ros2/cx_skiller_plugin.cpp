/***************************************************************************
 *  cx_skiller_plugin.cpp - Relay exec requests from ROS CX to the Skiller
 *
 *  Created: Oct 2023
 *  Copyright  2023  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "cx_skiller_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to act as Executor for the ROS 2 CLIPS-Executive
 * @author Tarik Viehmann
 */
class ROS2CXSkillerPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
    * @param config Fawkes configuration
    */
	explicit ROS2CXSkillerPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2CXSkillerThread());
	}
};

PLUGIN_DESCRIPTION("ROS CX Skiller Interface Plugin")
EXPORT_PLUGIN(ROS2CXSkillerPlugin)
