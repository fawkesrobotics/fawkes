
/***************************************************************************
 *  pcl_db_retrieve_plugin.cpp - Restore and retrieve PCLs from MongoDB
 *
 *  Created: Thu Aug 22 12:04:29 2013
 *  Copyright  2012-2013  Tim Niemueller [www.niemueller.de]
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

#include "pcl_db_retrieve_thread.h"

using namespace fawkes;

/** Plugin to segment a tabletop via PCL.
 * @author Tim Niemueller
 */
class PointCloudDBRetrievePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  PointCloudDBRetrievePlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new PointCloudDBRetrieveThread());
  }
};

PLUGIN_DESCRIPTION("Retrieve and transform point cloud from MongoDB")
EXPORT_PLUGIN(PointCloudDBRetrievePlugin)
