/***************************************************************************
 *  execution_time_estimator_plugin.cpp - Estimate execution times
 *
 *  Created: Thu 23 Apr 2020 16:36:22 CEST 16:36
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

#include <core/plugin.h>

/** @class ExecutionTimeEstimatorsPlugin
 *  Estimate skill execution times.
 *  This plugin provides an aspect that allows estimators to register.
 *
 *  @author Till Hofmann
 */

class ExecutionTimeEstimatorsPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
     *  @param config Fawkes configuration to read config values from.
     */
	explicit ExecutionTimeEstimatorsPlugin(fawkes::Configuration *config) : Plugin(config)
	{
		// TODO: initialize thread
	}
};

PLUGIN_DESCRIPTION("Provider for execution time estimates")
EXPORT_PLUGIN(ExecutionTimeEstimatorsPlugin)
