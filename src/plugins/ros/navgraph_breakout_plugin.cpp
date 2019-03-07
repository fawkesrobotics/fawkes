
/***************************************************************************
 *  navgraph_breakout_plugin.cpp - Provide navgraph-like API through ROS
 *
 *  Created: Fri Jan 27 11:19:24 2017
 *  Copyright  2017  Tim Niemueller
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

#include "navgraph_breakout_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Provide navgraph-like API through ROS.
 * @author Tim Niemueller
 */
class RosNavgraphBreakoutPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit RosNavgraphBreakoutPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RosNavgraphBreakoutThread());
	}
};

PLUGIN_DESCRIPTION("Use external ROS node for place goto")
EXPORT_PLUGIN(RosNavgraphBreakoutPlugin)
