
/***************************************************************************
 *  skiller_plugin.cpp - ROS Action Server to receive skiller commands from ROS
 *
 *  Created: Fri Jun 27 12:02:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "skiller_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Accept skiller commands from ROS.
 * @author Till Hofmann
 */
class RosSkillerPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit RosSkillerPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RosSkillerThread());
	}
};

PLUGIN_DESCRIPTION("Accept skiller commands from ROS")
EXPORT_PLUGIN(RosSkillerPlugin)
