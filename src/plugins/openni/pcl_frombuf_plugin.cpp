
/***************************************************************************
 *  pc_frombuf_plugin.cpp - Create PCL point clouds from OpenNI buffer
 *
 *  Created: Fri Dec 02 19:53:18 2011
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

#include <core/plugin.h>
#include "pcl_frombuf_thread.h"

using namespace fawkes;

/** Create PCL from pointcloud buffer.
 * @author Tim Niemueller
 */
class OpenNiPclOnlyPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenNiPclOnlyPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new OpenNiPclOnlyThread());
  }
};


PLUGIN_DESCRIPTION("Generate PCL pointclouds from buffer")
EXPORT_PLUGIN(OpenNiPclOnlyPlugin)

