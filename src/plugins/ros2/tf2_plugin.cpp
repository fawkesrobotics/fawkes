
/***************************************************************************
 *  ros_tf_plugin.cpp - Exchange transforms between Fawkes and ROS
 *
 *  Created: Wed Oct 26 00:45:26 2011
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

#include "tf2_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin exchange transforms between Fawkes and ROS.
 * @author Tim Niemueller
 */
class ROS2TFPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2TFPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2TF2Thread());
	}
};

PLUGIN_DESCRIPTION("ROS2 tf exchange plugin")
EXPORT_PLUGIN(ROS2TFPlugin)
