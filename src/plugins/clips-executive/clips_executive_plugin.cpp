
/***************************************************************************
 *  clips_executive_plugin.cpp - CLIPS Executive
 *
 *  Created: Tue Sep 19 11:51:53 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include "clips_executive_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS executive plugin.
 * @author Tim Niemueller
 */
class ClipsExecutivePlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	ClipsExecutivePlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ClipsExecutiveThread());
	}
};


PLUGIN_DESCRIPTION("CLIPS Executive")
EXPORT_PLUGIN(ClipsExecutivePlugin)
