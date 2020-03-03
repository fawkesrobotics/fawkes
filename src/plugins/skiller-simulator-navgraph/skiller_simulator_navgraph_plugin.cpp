/***************************************************************************
 *  skiller_simulator_navgraph_plugin.cpp - Skill exec times from navgraph
 *
 *  Created: Tue 07 Jan 2020 15:36:35 CET 15:36
 *  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "skiller_simulator_navgraph_thread.h"

#include <core/plugin.h>

/** @class SkillerSimulatorNavgraphPlugin
 * Plugin to get estimates for skill execution times from the navgraph.
 */

class SkillerSimulatorNavgraphPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config The fawkes config to use
   */
	explicit SkillerSimulatorNavgraphPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new SkillerSimulatorNavgraphEstimatorThread());
	}
};

PLUGIN_DESCRIPTION("Estimate skill execution times with the navgraph")
EXPORT_PLUGIN(SkillerSimulatorNavgraphPlugin)
