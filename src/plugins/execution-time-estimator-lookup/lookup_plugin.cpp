/***************************************************************************
 *  lookup_plugin.cpp - Get skill exec times from db lookups
 *
 *  Created: Tue 24 Jan 2020 09:36:35 CET 09:36
 *  Copyright  2020  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "lookup_thread.h"

#include <core/plugin.h>

/** @class ExecutionTimeEstimatorLookupPlugin
 * Plugin to get estimates for skill execution times from samples of a mongodb
 * database.
 */

class ExecutionTimeEstimatorLookupPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config The fawkes config to use
   */
	explicit ExecutionTimeEstimatorLookupPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ExecutionTimeEstimatorLookupEstimatorThread());
	}
};

PLUGIN_DESCRIPTION("Sample skill times from Mongodb Database")
EXPORT_PLUGIN(ExecutionTimeEstimatorLookupPlugin)
