/***************************************************************************
 *  skiller_simulator_plugin.cpp - Simulate the execution of a skill
 *
 *  Created: Mon 06 May 2019 09:01:14 CEST 09:01
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "skiller_simulator_plugin.h"

#include "exec_thread.h"

using namespace fawkes;

/** @class SkillerSimulatorPlugin <plugins/skiller-simulator/skiller_simulator_plugin.cpp>
 *  Skill Simulated Execution Plugin.
 *  This plugin acts like the skiller plugin but only simulates skill execution.
 *
 *  @author Till Hofmann
 */

/** Constructor.
 *  @param config Fawkes configuration to read config values from.
 */
SkillerSimulatorPlugin::SkillerSimulatorPlugin(Configuration *config) : Plugin(config)
{
	thread_list.push_back(new SkillerSimulatorExecutionThread());
}

PLUGIN_DESCRIPTION("Simulated Skill Execution")
EXPORT_PLUGIN(SkillerSimulatorPlugin)
