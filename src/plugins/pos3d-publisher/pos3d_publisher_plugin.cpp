
/***************************************************************************
 *  pos3d_publisher_plugin.cpp - a plugin to publish 3D robot positions
 *
 *  Created: Sat Apr 06 16:20:00 2024
 *  Copyright  2024  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#include "pos3d_publisher_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin to publish poses for the agent
 * @author Daniel Swoboda
 */
class Pos3dPublisherPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit Pos3dPublisherPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new Pos3dPublisherThread());
	}
};

PLUGIN_DESCRIPTION("pos3d publisher plugin")
EXPORT_PLUGIN(Pos3dPublisherPlugin)
