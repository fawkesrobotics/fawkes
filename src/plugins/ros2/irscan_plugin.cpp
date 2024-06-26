
/***************************************************************************
 *  irscan_plugin.cpp - Exchange irsensor data between Fawkes and ROS
 *
 *  Created: Mon Jul 03 13:41:18 2012
 *  Copyright  2023 Saurabh Borse, Tim Wendt
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

#include "irscan_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin exchange laser scans between Fawkes and ROS.
 * @author Tim Niemueller
 */
class Ros2IrScanPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit Ros2IrScanPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2IrScanThread());
	}
};

PLUGIN_DESCRIPTION("ROS IR scan exchange plugin")
EXPORT_PLUGIN(Ros2IrScanPlugin)
