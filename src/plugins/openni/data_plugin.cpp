
/***************************************************************************
 *  data_plugin.cpp - OpenNI raw data provider for Fawkes
 *
 *  Created: Thu Dec 22 11:31:32 2011
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

#include "image_thread.h"
#include "depth_thread.h"
#include "pointcloud_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin provide raw OpenNI data to Fawkes plugins.
 * This plugin publishes the image, depth, and point cloud data from
 * OpenNI to other Fawkes plugins.
 * @author Tim Niemueller
 */
class OpenNiDataPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenNiDataPlugin(Configuration *config) : Plugin(config)
  {
    OpenNiImageThread *img_thread = new OpenNiImageThread();
    thread_list.push_back(img_thread);
    thread_list.push_back(new OpenNiDepthThread());
    thread_list.push_back(new OpenNiPointCloudThread(img_thread));
  }
};

PLUGIN_DESCRIPTION("Image, depth, and point cloud provider")
EXPORT_PLUGIN(OpenNiDataPlugin)
