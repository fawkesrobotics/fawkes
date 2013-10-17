
/***************************************************************************
 *  pcl_db_merge_plugin_roscomm.cpp - ROS communication for pcl-db-merge
 *
 *  Created: Thu Dec 06 13:51:31 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "pcl_db_roscomm_thread.h"

using namespace fawkes;

/** Plugin to segment a tabletop via PCL.
 * @author Tim Niemueller
 */
class PointCloudDBROSCommPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  PointCloudDBROSCommPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new PointCloudDBROSCommThread());
  }
};

PLUGIN_DESCRIPTION("ROS communication for pcl-db plugins")
EXPORT_PLUGIN(PointCloudDBROSCommPlugin)

