
/***************************************************************************
 *  pcl_plugin.cpp - Exchange point clouds between Fawkes and ROS
 *
 *  Created: Mon Nov 07 02:21:36 2011
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

#include "pcl_thread.h"

#include <core/plugin.h>

using namespace fawkes;

/** Plugin exchange transforms between Fawkes and ROS.
 * @author Tim Niemueller
 */
class ROS2PointCloudPlugin : public fawkes::Plugin
{
public:
	/** Constructor.
   * @param config Fawkes configuration
   */
	explicit ROS2PointCloudPlugin(Configuration *config) : Plugin(config)
	{
		thread_list.push_back(new ROS2PointCloudThread());
	}
};

PLUGIN_DESCRIPTION("ROS point cloud exchange plugin")
EXPORT_PLUGIN(ROS2PointCloudPlugin)
