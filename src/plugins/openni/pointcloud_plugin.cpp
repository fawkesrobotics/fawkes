
/***************************************************************************
 *  pointcloud_plugin.cpp - Plugin to provide point clouds using OpenNI
 *
 *  Created: Fri Mar 25 23:45:39 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "pointcloud_plugin.h"
#include "pointcloud_thread.h"

using namespace fawkes;

/** @class OpenNiPointCloudPlugin "pointcloud_plugin.h"
 * Plugin to provide point clouds acquired using OpenNI.
 * This plugin uses OpenNI to acquire depth images from a camera and calculate
 * point clouds and provide them them via a SharedMemoryImageBuffer.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
OpenNiPointCloudPlugin::OpenNiPointCloudPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new OpenNiPointCloudThread());
}


PLUGIN_DESCRIPTION("Calculate point clouds using OpenNI")
EXPORT_PLUGIN(OpenNiPointCloudPlugin)

