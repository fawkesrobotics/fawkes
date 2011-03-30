
/***************************************************************************
 *  image_plugin.cpp - Plugin to provide images acquired using OpenNI
 *
 *  Created: Thu Mar 17 13:40:49 2011
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

#include "image_plugin.h"
#include "image_thread.h"

using namespace fawkes;

/** @class OpenNiImagePlugin "image_plugin.h"
 * Plugin to provide images acquired using OpenNI.
 * This plugin uses OpenNI to acquire images from a camera and provide them
 * via a SharedMemoryImageBuffer.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
OpenNiImagePlugin::OpenNiImagePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new OpenNiImageThread());
}


PLUGIN_DESCRIPTION("Acquire images from PrimeSense/Kinect using OpenNI")
EXPORT_PLUGIN(OpenNiImagePlugin)

