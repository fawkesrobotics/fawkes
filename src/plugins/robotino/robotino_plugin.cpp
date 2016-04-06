
/***************************************************************************
 *  robotino_plugin.cpp - Plugin for Robotino platform support
 *
 *  Created: Sun Nov 13 15:31:57 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <core/plugin.h>

#ifdef HAVE_OPENROBOTINO
#  include "openrobotino_com_thread.h"
#endif
#ifdef HAVE_ROBOTINO_DIRECT
#  include "direct_com_thread.h"
#endif
#include "sensor_thread.h"
#include "act_thread.h"

using namespace fawkes;

/** Plugin to provide Robotino platform support for Fawkes.
 * @author Tim Niemueller
 */
class RobotinoPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	RobotinoPlugin(Configuration *config)
		: Plugin(config)
	{
		std::string cfg_driver = config->get_string("/hardware/robotino/driver");
	  
		RobotinoComThread *com_thread = NULL;

		if (cfg_driver == "openrobotino") {
#ifdef HAVE_OPENROBOTINO
			com_thread = new OpenRobotinoComThread();
#else
			throw Exception("robotino: driver mode 'openrobotino' not available at compile time");
#endif
		} else if (cfg_driver == "direct") {
#ifdef HAVE_ROBOTINO_DIRECT
			com_thread = new DirectRobotinoComThread();    
#else
			throw Exception("robotino: driver mode 'direct' not available at compile time");
#endif
		} else {
			throw Exception("robotino: unknown driver '%s'", cfg_driver.c_str());
		}
		thread_list.push_back(com_thread);
		thread_list.push_back(new RobotinoSensorThread(com_thread));
		thread_list.push_back(new RobotinoActThread(com_thread));
	}
};

PLUGIN_DESCRIPTION("Robotino platform support")
EXPORT_PLUGIN(RobotinoPlugin)
