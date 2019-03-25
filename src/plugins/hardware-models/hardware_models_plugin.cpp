
/***************************************************************************
 *  hardware_models_plugin.cpp - Hardware Models
 *
 *  Created: Sun Mar 24 11:51:53 2019
 *  Copyright  2019 Daniel Habering (daniel@habering.de)
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

#include "hardware_models_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Hardware Models plugin.
 * @author Daniel Habering
 */
class HardwareModelsPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
	 * @param config Fawkes configuration
	 */
	explicit HardwareModelsPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new HardwareModelsThread());
	}
};


PLUGIN_DESCRIPTION("Hardware Models")
EXPORT_PLUGIN(HardwareModelsPlugin)
