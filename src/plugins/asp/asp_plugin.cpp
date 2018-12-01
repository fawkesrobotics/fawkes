
/***************************************************************************
 *  asp_plugin.cpp - Plugin to access ASP features
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
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

#include "asp_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin to access ASP from Fawkes.
 * This plugin integrates ASP and provides access to the Clingo context to other plugins.
 * @author Björn Schäpers
 */
class ASPPlugin : public fawkes::Plugin
{
	public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit ASPPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ASPThread());
	}
};


PLUGIN_DESCRIPTION("Provides Clingo environments")
EXPORT_PLUGIN(ASPPlugin)
