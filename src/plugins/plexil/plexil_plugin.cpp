
/***************************************************************************
 *  plexil_plugin.cpp - PLEXIL Executive Plugin
 *
 *  Created: Mon Aug 13 11:18:25 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "plexil_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** PLEXIL executive plugin.
 * @author Tim Niemueller
 */
class PlexilExecutivePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit PlexilExecutivePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new PlexilExecutiveThread());
	}
};


PLUGIN_DESCRIPTION("PLEXIL Executive")
EXPORT_PLUGIN(PlexilExecutivePlugin)
