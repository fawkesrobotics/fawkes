/***************************************************************************
 *  rl_test_plugin.cpp - 
 *
 *  Created: 
 *  Copyright  2020  
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

#include "rl-test-thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** @class RLTestPlugin
 *  Calls a python script
 *
 *  @author 
 */

class ClipsExecutiveRlPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
     *  @param config Fawkes configuration to read config values from.
     */
	explicit ClipsExecutiveRlPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new RLTestThread());
	}
};

PLUGIN_DESCRIPTION("Clips Executive RL Agent")
EXPORT_PLUGIN(ClipsExecutiveRlPlugin)
